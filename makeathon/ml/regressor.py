"""
Fertilizer dosage predictor — XGBoost regression on synthetic training data.

Features
--------
    hour              (int,   0–23)     time of day
    temp_c            (float, °C)       ambient temperature
    humidity_pct      (float, %)        relative humidity
    vpd_kpa           (float, kPa)      vapour pressure deficit (derived)
    avg_stem_width_px (float, px)       mean bounding-box width of detected stems
    species           (int,   0–2)      0 = anthurium, 1 = basil, 2 = money_tree

Target
------
    dispense_seconds  (float, 5–60)     fertilizer pump run duration

The model is trained once on synthetic data and cached to data/fertilizer_model.pkl.
On every subsequent run the cached model is loaded instantly; no re-training occurs.

Usage
-----
    from makeathon.ml.regressor import FertilizerPredictor

    predictor = FertilizerPredictor()
    seconds = predictor.predict(
        hour=9, temp_c=24.0, humidity_pct=65.0,
        avg_stem_width_px=42.0, species="money_tree"
    )
    # → e.g. 21.4
"""

import math
import pickle
from pathlib import Path

import numpy as np

# ── Constants ─────────────────────────────────────────────────────────────────

MODEL_PATH = Path(__file__).resolve().parents[2] / "data" / "fertilizer_model.pkl"

SPECIES_MAP: dict[str, int] = {
    "anthurium":  0,
    "basil":      1,
    "money_tree": 2,
}

# Base dispense durations (seconds) per species — before regression adjustment
BASE_DOSE: dict[str, float] = {
    "anthurium":  25.0,
    "basil":      15.0,
    "money_tree": 20.0,
}

DISPENSE_MIN = 5.0
DISPENSE_MAX = 60.0


# ── Helpers ───────────────────────────────────────────────────────────────────

def vpd_kpa(temp_c: float, humidity_pct: float) -> float:
    """Vapour Pressure Deficit in kPa (Tetens / Buck equation)."""
    svp = 0.6108 * math.exp(17.27 * temp_c / (temp_c + 237.3))
    return round(svp * (1.0 - humidity_pct / 100.0), 4)


def _vpd_array(temps: np.ndarray, humidities: np.ndarray) -> np.ndarray:
    svp = 0.6108 * np.exp(17.27 * temps / (temps + 237.3))
    return svp * (1.0 - humidities / 100.0)


# ── Synthetic data generation ─────────────────────────────────────────────────

def generate_synthetic_data(n: int = 800, seed: int = 42) -> tuple[np.ndarray, np.ndarray]:
    """
    Generate n biologically motivated training samples.

    The target is constructed from known agronomic relationships so the
    model learns a meaningful, generalisable function rather than pure noise.

    Returns
    -------
    X : ndarray, shape (n, 6)
        Feature matrix [hour, temp_c, humidity_pct, vpd_kpa,
                         avg_stem_width_px, species_idx]
    y : ndarray, shape (n,)
        Dispense duration in seconds, clipped to [DISPENSE_MIN, DISPENSE_MAX]
    """
    rng = np.random.default_rng(seed)

    # ── Raw features ──────────────────────────────────────────────────────────
    hours       = rng.integers(0, 24, n).astype(float)
    temps       = rng.uniform(18.0, 35.0, n)
    humidities  = rng.uniform(40.0, 90.0, n)
    vpds        = _vpd_array(temps, humidities)
    stem_widths = rng.uniform(10.0, 80.0, n)

    species_names  = list(SPECIES_MAP.keys())
    species_labels = rng.choice(species_names, n)
    species_idx    = np.array([SPECIES_MAP[s] for s in species_labels])
    base_doses     = np.array([BASE_DOSE[s]   for s in species_labels])

    # ── Agronomic modifiers ───────────────────────────────────────────────────
    # High VPD → plant transpires more → more fertilizer needed
    vpd_factor = 1.0 + np.clip((vpds - 1.0) * 0.20, -0.25, 0.40)

    # Optimal watering windows: early morning (8 am) and late afternoon (17 h)
    # Cosine centred on 8 am; secondary lift in the evening approximated by offset
    time_factor = (
        1.0
        + 0.10 * np.cos(2 * np.pi * (hours - 8.0)  / 24.0)   # morning peak
        + 0.05 * np.cos(2 * np.pi * (hours - 17.0) / 24.0)   # evening lift
    )

    # Larger plant → more biomass → higher fertilizer requirement
    width_factor = 0.70 + (stem_widths / 80.0) * 0.60

    # Mild temperature boost above 26 °C (heat stress increases demand)
    temp_factor = 1.0 + np.clip((temps - 26.0) * 0.012, -0.10, 0.18)

    # ── Target ────────────────────────────────────────────────────────────────
    dispense = (
        base_doses
        * vpd_factor
        * time_factor
        * width_factor
        * temp_factor
        + rng.normal(0.0, 1.5, n)          # realistic sensor / environment noise
    )
    dispense = np.clip(dispense, DISPENSE_MIN, DISPENSE_MAX).astype(np.float32)

    X = np.column_stack([hours, temps, humidities, vpds, stem_widths, species_idx])
    return X, dispense


# ── Training ──────────────────────────────────────────────────────────────────

def train() -> object:
    """
    Train an XGBRegressor on synthetic data, persist it, and return it.

    The trained model is saved to data/fertilizer_model.pkl so subsequent
    runs skip training entirely.
    """
    from xgboost import XGBRegressor

    print("Training fertilizer dosage model on synthetic data...")
    X, y = generate_synthetic_data()

    # Reproducible shuffle-split (80 / 20)
    rng  = np.random.default_rng(0)
    perm = rng.permutation(len(X))
    split = int(len(X) * 0.80)
    tr, te = perm[:split], perm[split:]

    model = XGBRegressor(
        n_estimators=300,
        max_depth=4,
        learning_rate=0.05,
        subsample=0.8,
        colsample_bytree=0.8,
        random_state=42,
        verbosity=0,
    )
    model.fit(X[tr], y[tr])

    # R² on hold-out set
    preds  = model.predict(X[te])
    ss_res = float(np.sum((y[te] - preds) ** 2))
    ss_tot = float(np.sum((y[te] - y[te].mean()) ** 2))
    r2     = 1.0 - ss_res / ss_tot
    print(f"Model trained. Hold-out R² = {r2:.3f}")

    MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(MODEL_PATH, "wb") as fh:
        pickle.dump(model, fh)
    print(f"Model saved to {MODEL_PATH}")

    return model


# ── Public predictor ──────────────────────────────────────────────────────────

class FertilizerPredictor:
    """
    Lazy-loading wrapper around the trained XGBRegressor.

    The model is loaded from disk on the first call to predict().
    If no saved model exists, it is trained automatically.
    """

    def __init__(self) -> None:
        self._model = None

    def _load(self) -> None:
        if self._model is not None:
            return
        if MODEL_PATH.exists():
            with open(MODEL_PATH, "rb") as fh:
                self._model = pickle.load(fh)
        else:
            self._model = train()

    def predict(
        self,
        hour: int,
        temp_c: float,
        humidity_pct: float,
        avg_stem_width_px: float,
        species: str,
    ) -> float:
        """
        Predict fertilizer dispense duration.

        Parameters
        ----------
        hour              : 0–23
        temp_c            : ambient temperature in °C
        humidity_pct      : relative humidity 0–100
        avg_stem_width_px : mean width of detected stem bounding boxes (pixels)
        species           : one of 'anthurium', 'basil', 'money_tree'

        Returns
        -------
        float : pump run duration in seconds, clamped to [5, 60]
        """
        self._load()

        vpd        = vpd_kpa(temp_c, humidity_pct)
        species_id = SPECIES_MAP.get(species, SPECIES_MAP["basil"])

        X = np.array([[hour, temp_c, humidity_pct, vpd, avg_stem_width_px, species_id]],
                     dtype=np.float32)

        raw = float(self._model.predict(X)[0])
        return round(max(DISPENSE_MIN, min(DISPENSE_MAX, raw)), 1)
