#!/usr/bin/env python3
"""
===============================================================================
Project: Vision-Based Gesture-Controlled Person-Following Robot
File: vision/train_gesture_model.py
Description: Trains a Random Forest Classifier on normalized 63D hand features
             with open-set k-NN distance threshold boundaries for outlier rejection.
===============================================================================
"""

import os
import glob
import joblib
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, classification_report

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
GESTURE_DATA_DIR = os.path.join(SCRIPT_DIR, "gesture_data")
MODEL_PATH = os.path.join(SCRIPT_DIR, "gesture_model.pkl")

GESTURES = ["FOLLOW", "STOP", "DOCK"]
K_NEIGHBORS = 5
VALIDATION_SIZE = 0.20
RANDOM_STATE = 42
BASE_PERCENTILE = 95
LIVE_TOLERANCE_MULTIPLIER = 3.5
MIN_DISTANCE_THRESHOLD = 0.25


def generate_synthetic_samples(gesture_name, count=50):
    """Generates synthetic baseline samples if user hasn't recorded data yet."""
    np.random.seed(42 + GESTURES.index(gesture_name))
    center = np.random.uniform(-0.5, 0.5, 63).astype(np.float32)
    noise = np.random.normal(0, 0.05, (count, 63)).astype(np.float32)
    return center + noise


def load_dataset():
    features = []
    labels = []

    for gesture in GESTURES:
        path = os.path.join(GESTURE_DATA_DIR, f"{gesture}.csv")
        samples = []
        if os.path.exists(path):
            with open(path, "r") as f:
                for line in f:
                    parts = line.strip().split(",")
                    try:
                        vec = [float(p) for p in parts]
                        if len(vec) == 63:
                            samples.append(vec)
                    except ValueError:
                        continue

        if len(samples) < 10:
            print(f"[WARN] Insufficient samples for {gesture}. Generating synthetic baseline.")
            samples = generate_synthetic_samples(gesture).tolist()

        features.extend(samples)
        labels.extend([gesture] * len(samples))
        print(f"[OK] {gesture}: Loaded {len(samples)} samples.")

    return np.array(features, dtype=np.float32), np.array(labels)


def compute_knn_thresholds(features, labels, k_val):
    thresholds = {}
    for gesture in GESTURES:
        class_feat = features[labels == gesture]
        distances = []
        for i, sample in enumerate(class_feat):
            rest = np.delete(class_feat, i, axis=0)
            dists = np.linalg.norm(rest - sample, axis=1)
            k = min(k_val, len(dists))
            distances.append(np.mean(np.sort(dists)[:k]))

        perc = np.percentile(distances, BASE_PERCENTILE)
        adaptive = max(MIN_DISTANCE_THRESHOLD, perc * LIVE_TOLERANCE_MULTIPLIER)
        thresholds[gesture] = float(adaptive)
        print(f"  {gesture}: P{BASE_PERCENTILE}={perc:.4f} -> Threshold={thresholds[gesture]:.4f}")
    return thresholds


def main():
    print("=" * 60)
    print("  TRAINING GESTURE MODEL WITH OPEN-SET REJECTION BOUNDARIES")
    print("=" * 60)

    X, y = load_dataset()
    X_train, X_val, y_train, y_val = train_test_split(
        X, y, test_size=VALIDATION_SIZE, random_state=RANDOM_STATE, stratify=y
    )

    clf = RandomForestClassifier(n_estimators=100, max_depth=12, random_state=RANDOM_STATE)
    clf.fit(X_train, y_train)

    preds = clf.predict(X_val)
    acc = accuracy_score(y_val, preds)
    print(f"\n[OK] Validation Accuracy: {acc*100:.2f}%\n")
    print(classification_report(y_val, preds))

    print("\nComputing adaptive distance thresholds...")
    thresholds = compute_knn_thresholds(X, y, K_NEIGHBORS)

    model_payload = {
        "model": clf,
        "training_features": X,
        "training_labels": y,
        "distance_thresholds": thresholds,
        "distance_k": K_NEIGHBORS,
    }

    joblib.dump(model_payload, MODEL_PATH)
    print(f"\n[SUCCESS] Model successfully serialized to: {MODEL_PATH}")


if __name__ == "__main__":
    main()