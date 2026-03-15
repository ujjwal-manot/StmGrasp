#!/usr/bin/env python3
"""Train material classifiers and export as C headers for STM32."""

import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.naive_bayes import GaussianNB
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, accuracy_score
from sklearn.tree import export_text
import os

MATERIALS = {
    "Metal":     {"log_mag": (0.0, 0.6), "phase": (0, 10),   "freq": (1800, 600), "decay": (0.35, 0.12)},
    "Skin":      {"log_mag": (4.7, 0.5), "phase": (-35, 12), "freq": (100, 80),   "decay": (0.01, 0.02)},
    "Plastic":   {"log_mag": (6.0, 0.5), "phase": (-85, 8),  "freq": (900, 400),  "decay": (0.12, 0.08)},
    "Wood":      {"log_mag": (5.7, 0.5), "phase": (-20, 12), "freq": (350, 200),  "decay": (0.06, 0.05)},
    "Glass":     {"log_mag": (7.0, 0.4), "phase": (-88, 5),  "freq": (2800, 800), "decay": (0.30, 0.10)},
    "Cardboard": {"log_mag": (5.3, 0.5), "phase": (-25, 12), "freq": (200, 120),  "decay": (0.03, 0.03)},
}

FEATURE_NAMES = ["log_magnitude", "phase", "dominant_freq", "decay_ratio"]
CLASS_NAMES = list(MATERIALS.keys())
N_SAMPLES = 200

def generate_data():
    X, y = [], []
    for label, (name, params) in enumerate(MATERIALS.items()):
        samples = np.column_stack([
            np.random.normal(params["log_mag"][0], params["log_mag"][1], N_SAMPLES),
            np.random.normal(params["phase"][0], params["phase"][1], N_SAMPLES),
            np.random.normal(params["freq"][0], params["freq"][1], N_SAMPLES),
            np.random.normal(params["decay"][0], params["decay"][1], N_SAMPLES),
        ])
        X.append(samples)
        y.extend([label] * N_SAMPLES)
    return np.vstack(X), np.array(y)

def tree_to_c(tree, tree_idx, feature_names):
    tree_ = tree.tree_
    lines = []
    def recurse(node, depth):
        indent = "    " * (depth + 1)
        if tree_.feature[node] != -2:
            feat = feature_names[tree_.feature[node]]
            thresh = tree_.threshold[node]
            lines.append(f"{indent}if (x[{tree_.feature[node]}] <= {thresh:.6f}f) {{")
            recurse(tree_.children_left[node], depth + 1)
            lines.append(f"{indent}}} else {{")
            recurse(tree_.children_right[node], depth + 1)
            lines.append(f"{indent}}}")
        else:
            class_idx = int(np.argmax(tree_.value[node]))
            lines.append(f"{indent}return {class_idx};")
    lines.append(f"static int rf_tree_{tree_idx}(const float x[{len(feature_names)}]) {{")
    recurse(0, 0)
    lines.append("}")
    return "\n".join(lines)

def export_rf_header(clf, path):
    n_trees = len(clf.estimators_)
    n_classes = clf.n_classes_
    parts = [
        "#ifndef RF_MODEL_H",
        "#define RF_MODEL_H",
        "",
        f"#define RF_N_TREES {n_trees}",
        f"#define RF_N_CLASSES {n_classes}",
        f"#define RF_N_FEATURES {len(FEATURE_NAMES)}",
        "",
        "static const char* rf_class_names[] = {" + ", ".join(f'"{n}"' for n in CLASS_NAMES) + "};",
        "",
    ]
    for i, est in enumerate(clf.estimators_):
        parts.append(tree_to_c(est, i, FEATURE_NAMES))
        parts.append("")

    parts.append(f"typedef int (*rf_tree_fn)(const float x[RF_N_FEATURES]);")
    parts.append(f"static rf_tree_fn rf_trees[RF_N_TREES] = {{")
    parts.append("    " + ", ".join(f"rf_tree_{i}" for i in range(n_trees)) + "};")
    parts.append("")
    parts.append("static int rf_predict(const float x[RF_N_FEATURES]) {")
    parts.append("    int votes[RF_N_CLASSES] = {0};")
    parts.append("    for (int i = 0; i < RF_N_TREES; i++) votes[rf_trees[i](x)]++;")
    parts.append("    int best = 0;")
    parts.append("    for (int i = 1; i < RF_N_CLASSES; i++)")
    parts.append("        if (votes[i] > votes[best]) best = i;")
    parts.append("    return best;")
    parts.append("}")
    parts.append("")
    parts.append("#endif")

    with open(path, "w") as f:
        f.write("\n".join(parts) + "\n")
    print(f"Exported RandomForest -> {path}")

def export_nb_header(clf, path):
    n_classes = len(clf.classes_)
    n_features = clf.theta_.shape[1]
    parts = [
        "#ifndef NB_MODEL_H",
        "#define NB_MODEL_H",
        "",
        "#include <math.h>",
        "",
        f"#define NB_N_CLASSES {n_classes}",
        f"#define NB_N_FEATURES {n_features}",
        "",
        "static const char* nb_class_names[] = {" + ", ".join(f'"{n}"' for n in CLASS_NAMES) + "};",
        "",
        "static const float nb_mean[NB_N_CLASSES][NB_N_FEATURES] = {",
    ]
    for i in range(n_classes):
        vals = ", ".join(f"{v:.6f}f" for v in clf.theta_[i])
        parts.append(f"    {{{vals}}},")
    parts.append("};")
    parts.append("")

    std = np.sqrt(clf.var_)
    parts.append("static const float nb_std[NB_N_CLASSES][NB_N_FEATURES] = {")
    for i in range(n_classes):
        vals = ", ".join(f"{v:.6f}f" for v in std[i])
        parts.append(f"    {{{vals}}},")
    parts.append("};")
    parts.append("")

    parts.append("static const float nb_log_prior[NB_N_CLASSES] = {")
    vals = ", ".join(f"{v:.6f}f" for v in clf.class_prior_)
    parts.append(f"    {vals}")
    parts.append("};")
    parts.append("")

    parts.append("static int nb_predict(const float x[NB_N_FEATURES]) {")
    parts.append("    float best_score = -1e30f;")
    parts.append("    int best_class = 0;")
    parts.append("    for (int c = 0; c < NB_N_CLASSES; c++) {")
    parts.append("        float score = logf(nb_log_prior[c]);")
    parts.append("        for (int f = 0; f < NB_N_FEATURES; f++) {")
    parts.append("            float diff = x[f] - nb_mean[c][f];")
    parts.append("            float s = nb_std[c][f];")
    parts.append("            score += -0.5f * (diff * diff) / (s * s) - logf(s);")
    parts.append("        }")
    parts.append("        if (score > best_score) { best_score = score; best_class = c; }")
    parts.append("    }")
    parts.append("    return best_class;")
    parts.append("}")
    parts.append("")
    parts.append("#endif")

    with open(path, "w") as f:
        f.write("\n".join(parts) + "\n")
    print(f"Exported NaiveBayes  -> {path}")

def main():
    np.random.seed(42)
    X, y = generate_data()
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42, stratify=y)

    rf = RandomForestClassifier(n_estimators=10, max_depth=8, random_state=42)
    rf.fit(X_train, y_train)
    rf_pred = rf.predict(X_test)

    nb = GaussianNB()
    nb.fit(X_train, y_train)
    nb_pred = nb.predict(X_test)

    print("=" * 60)
    print("RandomForest Classifier")
    print("=" * 60)
    print(f"Accuracy: {accuracy_score(y_test, rf_pred):.4f}")
    print(classification_report(y_test, rf_pred, target_names=CLASS_NAMES))

    print("=" * 60)
    print("Naive Bayes Classifier")
    print("=" * 60)
    print(f"Accuracy: {accuracy_score(y_test, nb_pred):.4f}")
    print(classification_report(y_test, nb_pred, target_names=CLASS_NAMES))

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "model_export")
    os.makedirs(out_dir, exist_ok=True)

    try:
        import emlearn
        cmodel = emlearn.convert(rf, method="inline")
        emlearn_path = os.path.join(out_dir, "rf_model_emlearn.h")
        cmodel.save(file=emlearn_path, name="rf_model")
        print(f"Exported via emlearn  -> {emlearn_path}")
    except ImportError:
        print("emlearn not installed, using manual tree export")

    export_rf_header(rf, os.path.join(out_dir, "rf_model.h"))
    export_nb_header(nb, os.path.join(out_dir, "nb_model.h"))

if __name__ == "__main__":
    main()
