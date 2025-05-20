import pandas as pd
import numpy as np
import os
import pickle
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler, LabelEncoder
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score
from sklearn.pipeline import Pipeline
from tqdm import tqdm

def load_feature_dataset(file_path='d:/Downloads/AI_ML/data/features/extracted_features.csv', 
                        exclude_activities=None):
    """
    Load the feature dataset and optionally exclude specific activities
    
    Args:
        file_path: Path to the feature dataset CSV file
        exclude_activities: List of activity types to exclude (e.g. ['walkFall'])
    
    Returns:
        DataFrame with features, excluding specified activities
    """
    print(f"Loading feature dataset from {file_path}")
    df = pd.read_csv(file_path)
    
    if exclude_activities:
        original_count = len(df)
        df = df[~df['ActivityType'].isin(exclude_activities)]
        excluded_count = original_count - len(df)
        print(f"Excluded {excluded_count} samples of {exclude_activities}")
        
        # Show remaining activity distribution
        activity_counts = df['ActivityType'].value_counts()
        print("\nRemaining activity distribution:")
        for activity, count in activity_counts.items():
            print(f"  {activity}: {count} samples")
    
    return df

def prepare_data(df, test_size=0.2, target_column='ActivityType'):
    """
    Prepare data for model training
    
    Args:
        df: DataFrame with features
        test_size: Fraction of data to use for testing
        target_column: Column to use as target ('IsFall' for binary, 'ActivityType' for multi-class)
    
    Returns:
        X_train, X_test, y_train, y_test, feature_names, label_encoder
    """
    # Remove non-feature columns
    non_feature_cols = ['IsFall', 'ActivityType', 'SourceFile']
    feature_cols = [col for col in df.columns if col not in non_feature_cols]
    
    # Split into features and target
    X = df[feature_cols]
    
    # For multi-class classification, we use ActivityType
    y = df[target_column]
    
    # If we're doing multi-class classification, encode the labels
    label_encoder = None
    if target_column == 'ActivityType':
        label_encoder = LabelEncoder()
        y = label_encoder.fit_transform(y)
        print("\nActivity Type Encoding:")
        for i, activity in enumerate(label_encoder.classes_):
            print(f"  {activity} -> {i}")
    
    # Split into training and testing sets
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=test_size, random_state=42, stratify=y)
    
    print(f"\nTraining set: {X_train.shape[0]} samples")
    print(f"Testing set: {X_test.shape[0]} samples")
    
    if target_column == 'ActivityType':
        # Print distribution of activity types
        print("\nActivity Type Distribution:")
        for i, activity in enumerate(label_encoder.classes_):
            train_count = sum(y_train == i)
            test_count = sum(y_test == i)
            print(f"  {activity}: {train_count} in training, {test_count} in testing")
    else:
        # Print fall/non-fall distribution
        print(f"Fall samples in training: {sum(y_train)}")
        print(f"Fall samples in testing: {sum(y_test)}")
    
    return X_train, X_test, y_train, y_test, feature_cols, label_encoder

def evaluate_model(model, X_test, y_test, label_encoder=None):
    """
    Evaluate a trained model and generate performance metrics
    
    Args:
        model: Trained model
        X_test: Test features
        y_test: Test labels
        label_encoder: Label encoder for activity types
    
    Returns:
        Dictionary of performance metrics
    """
    # Make predictions
    y_pred = model.predict(X_test)
    
    # Calculate metrics
    accuracy = accuracy_score(y_test, y_pred)
    
    # Print classification report
    print("\n=== Random Forest Performance ===")
    print(f"Accuracy: {accuracy:.4f}")
    print("\nClassification Report:")
    
    if label_encoder is not None:
        # Multi-class classification
        class_names = label_encoder.classes_
        print(classification_report(y_test, y_pred, target_names=class_names))
        
        # Plot confusion matrix
        cm = confusion_matrix(y_test, y_pred)
        plt.figure(figsize=(10, 8))
        sns.heatmap(cm, annot=True, fmt="d", cmap="Blues", 
                    xticklabels=class_names,
                    yticklabels=class_names)
        plt.xlabel("Predicted")
        plt.ylabel("Actual")
        plt.title("Confusion Matrix - Random Forest (Activity Types)")
        
        # Save the confusion matrix
        os.makedirs('d:/Downloads/AI_ML/models/results', exist_ok=True)
        plt.savefig("d:/Downloads/AI_ML/models/results/random_forest_multiclass_confusion_matrix.png")
        plt.close()
    else:
        # Binary classification
        print(classification_report(y_test, y_pred, target_names=["No Fall", "Fall"]))
        
        # Plot confusion matrix
        cm = confusion_matrix(y_test, y_pred)
        plt.figure(figsize=(8, 6))
        sns.heatmap(cm, annot=True, fmt="d", cmap="Blues", 
                    xticklabels=["No Fall", "Fall"],
                    yticklabels=["No Fall", "Fall"])
        plt.xlabel("Predicted")
        plt.ylabel("Actual")
        plt.title("Confusion Matrix - Random Forest (Fall Detection)")
        
        # Save the confusion matrix
        os.makedirs('d:/Downloads/AI_ML/models/results', exist_ok=True)
        plt.savefig("d:/Downloads/AI_ML/models/results/random_forest_binary_confusion_matrix.png")
        plt.close()
    
    # Feature importance
    feature_importances = model.named_steps['model'].feature_importances_
    feature_names = X_test.columns
    
    # Sort feature importances in descending order
    indices = np.argsort(feature_importances)[::-1]
    
    # Plot top 20 feature importances
    plt.figure(figsize=(12, 8))
    plt.title("Feature Importances (Random Forest)")
    plt.bar(range(min(20, len(feature_names))), feature_importances[indices][:20], align="center")
    plt.xticks(range(min(20, len(feature_names))), [feature_names[i] for i in indices][:20], rotation=90)
    plt.tight_layout()
    plt.savefig("d:/Downloads/AI_ML/models/results/random_forest_feature_importance.png")
    plt.close()
    
    return {
        'accuracy': accuracy
    }

def train_random_forest(X_train, y_train, X_test, y_test, n_classes=2):
    """
    Train a Random Forest classifier
    
    Args:
        X_train, y_train: Training data
        X_test, y_test: Testing data
        n_classes: Number of classes (2 for binary, >2 for multi-class)
    
    Returns:
        Trained model
    """
    print("\nTraining Random Forest...")
    
    # Create pipeline with scaling and model
    pipeline = Pipeline([
        ('scaler', StandardScaler()),
        ('model', RandomForestClassifier(
            n_estimators=100,
            max_depth=10,
            min_samples_split=2,
            min_samples_leaf=1,
            random_state=42,
            n_jobs=-1))
    ])
    
    # Train the model
    pipeline.fit(X_train, y_train)
    
    return pipeline

def save_model(model, label_encoder=None, output_dir='d:/Downloads/AI_ML/models'):
    """
    Save trained model to disk
    
    Args:
        model: Trained model
        label_encoder: Label encoder for activity types
        output_dir: Directory to save model
    """
    os.makedirs(output_dir, exist_ok=True)
    
    model_path = os.path.join(output_dir, "random_forest.pkl")
    with open(model_path, 'wb') as f:
        pickle.dump(model, f)
    print(f"Saved Random Forest model to {model_path}")
    
    # Save label encoder if provided
    if label_encoder is not None:
        encoder_path = os.path.join(output_dir, "label_encoder.pkl")
        with open(encoder_path, 'wb') as f:
            pickle.dump(label_encoder, f)
        print(f"Saved Label Encoder to {encoder_path}")

def convert_model_for_esp32(model, feature_names, label_encoder=None, output_dir='d:/Downloads/AI_ML/esp32_integration'):
    """
    Convert the trained model to C code for ESP32
    
    Args:
        model: Trained model
        feature_names: List of feature names
        label_encoder: Label encoder for activity types
        output_dir: Directory to save C code
    """
    os.makedirs(output_dir, exist_ok=True)
    
    print("\nConverting model to C code for ESP32...")
    
    # For Random Forest, extract a subset of trees
    rf_model = model.named_steps['model']
    scaler = model.named_steps['scaler']
    
    # Limit to 5 trees for ESP32
    n_trees = min(5, rf_model.n_estimators)
    
    # Determine if multi-class or binary
    n_classes = len(np.unique(rf_model.classes_))
    is_multiclass = n_classes > 2
    
    # Function to convert a decision tree to C code
    def tree_to_c_code(tree, depth=0):
        tree_ = tree.tree_
        feature = tree_.feature
        threshold = tree_.threshold
        
        code = []
        
        def recurse(node, depth):
            indent = "  " * depth
            
            if node < len(feature) and feature[node] != -2:  # Thêm kiểm tra node < len(feature)
                feature_idx = feature[node]
                # Đảm bảo feature_idx hợp lệ
                if feature_idx < len(feature_names):
                    feature_name = feature_names[feature_idx]
                    
                    code.append(f"{indent}if (features[{feature_idx}] <= {threshold[node]:.6f}f) {{")
                    # Kiểm tra node*2+1 có hợp lệ không
                    if node * 2 + 1 < len(feature):
                        recurse(node * 2 + 1, depth + 1)  # Left child
                    else:
                        code.append(f"{indent}  return 0;  // Safety check: Index out of bounds")
                    
                    code.append(f"{indent}}} else {{")
                    # Kiểm tra node*2+2 có hợp lệ không
                    if node * 2 + 2 < len(feature):
                        recurse(node * 2 + 2, depth + 1)  # Right child
                    else:
                        code.append(f"{indent}  return 0;  // Safety check: Index out of bounds")
                    
                    code.append(f"{indent}}}")
                else:
                    code.append(f"{indent}return 0;  // Safety check: Feature index out of bounds")
            else:
                # Get the class with highest probability
                if node < len(tree_.value):
                    class_values = tree_.value[node][0]
                    class_prediction = np.argmax(class_values)
                    code.append(f"{indent}return {int(class_prediction)};")
                else:
                    code.append(f"{indent}return 0;  // Safety check: Node index out of bounds")
        
        # Thử xử lý gốc, nếu gặp lỗi thì trả về hàm đơn giản
        try:
            recurse(0, depth)
            return '\n'.join(code)
        except Exception as e:
            print(f"Error generating C code for tree: {e}")
            # Trả về hàm đơn giản luôn trả về 0
            return "  return 0;  // Error in tree conversion"
    
    # Generate C code for each tree
    tree_functions = []
    for i in range(n_trees):
        tree = rf_model.estimators_[i]
        tree_code = tree_to_c_code(tree)
        tree_functions.append(f"int tree_{i}(float features[]) {{\n{tree_code}\n}}")
    
    # Generate detection function (different for multi-class vs binary)
    if is_multiclass:
        # For multi-class, determine the most voted class
        detection_function = """
int detect_activity(float features[]) {
  int votes[%d] = {0};
  int n_trees = %d;
  int i, max_votes = 0, predicted_class = 0;
  
  // Count votes for each class
  %s
  
  // Find class with most votes
  for (i = 0; i < %d; i++) {
    if (votes[i] > max_votes) {
      max_votes = votes[i];
      predicted_class = i;
    }
  }
  
  return predicted_class;
}

bool detect_fall(float features[]) {
  // In multi-class setup, classes 0-2 might be falls (depends on encoding)
  // This needs to be adjusted based on your specific label encoding
  int activity = detect_activity(features);
  
  // Example: if classes 0, 1, 2 are fall activities (runFall, walkFall, freeFall)
  return (activity >= 0 && activity <= 2);
}
""" % (
            n_classes,
            n_trees,
            '\n  '.join([f"votes[tree_{i}(features)]++;" for i in range(n_trees)]),
            n_classes
        )
    else:
        # For binary, just count votes for class 1 (fall)
        detection_function = """
bool detect_fall(float features[]) {
  int votes = 0;
  int n_trees = %d;
  
  %s
  
  return votes > (n_trees / 2);
}
""" % (n_trees, '\n  '.join([f"votes += tree_{i}(features);" for i in range(n_trees)]))
    
    # Generate feature extraction function (same for both types)
    feature_extraction = """
void extract_features(float accel_x[], float accel_y[], float accel_z[], 
                     int window_size, float features[]) {
  float mean_x = 0, mean_y = 0, mean_z = 0;
  float min_x = accel_x[0], min_y = accel_y[0], min_z = accel_z[0];
  float max_x = accel_x[0], max_y = accel_y[0], max_z = accel_z[0];
  
  for (int i = 0; i < window_size; i++) {
    mean_x += accel_x[i];
    mean_y += accel_y[i];
    mean_z += accel_z[i];
    
    if (accel_x[i] < min_x) min_x = accel_x[i];
    if (accel_y[i] < min_y) min_y = accel_y[i];
    if (accel_z[i] < min_z) min_z = accel_z[i];
    
    if (accel_x[i] > max_x) max_x = accel_x[i];
    if (accel_y[i] > max_y) max_y = accel_y[i];
    if (accel_z[i] > max_z) max_z = accel_z[i];
  }
  
  mean_x /= window_size;
  mean_y /= window_size;
  mean_z /= window_size;
  
  float var_x = 0, var_y = 0, var_z = 0;
  for (int i = 0; i < window_size; i++) {
    var_x += (accel_x[i] - mean_x) * (accel_x[i] - mean_x);
    var_y += (accel_y[i] - mean_y) * (accel_y[i] - mean_y);
    var_z += (accel_z[i] - mean_z) * (accel_z[i] - mean_z);
  }
  
  var_x /= window_size;
  var_y /= window_size;
  var_z /= window_size;
  
  float std_x = sqrt(var_x);
  float std_y = sqrt(var_y);
  float std_z = sqrt(var_z);
  
  float mag_values[window_size];
  float mean_mag = 0, min_mag = 0, max_mag = 0;
  
  for (int i = 0; i < window_size; i++) {
    mag_values[i] = sqrt(accel_x[i]*accel_x[i] + accel_y[i]*accel_y[i] + accel_z[i]*accel_z[i]);
    mean_mag += mag_values[i];
    
    if (i == 0 || mag_values[i] < min_mag) min_mag = mag_values[i];
    if (i == 0 || mag_values[i] > max_mag) max_mag = mag_values[i];
  }
  
  mean_mag /= window_size;
  
  int idx = 0;
  features[idx++] = mean_x;
  features[idx++] = std_x;
  features[idx++] = min_x;
  features[idx++] = max_x;
  features[idx++] = max_x - min_x;
  
  features[idx++] = mean_y;
  features[idx++] = std_y;
  features[idx++] = min_y;
  features[idx++] = max_y;
  features[idx++] = max_y - min_y;
  
  features[idx++] = mean_z;
  features[idx++] = std_z;
  features[idx++] = min_z;
  features[idx++] = max_z;
  features[idx++] = max_z - min_z;
  
  features[idx++] = mean_mag;
  features[idx++] = max_mag;
  features[idx++] = max_mag - min_mag;
}
"""
    
    # Add activity names as comments (helps with debugging)
    activity_map = ""
    if label_encoder is not None:
        activity_map = "\n// Activity type mapping:\n"
        for i, activity in enumerate(label_encoder.classes_):
            activity_map += f"// {i}: {activity}\n"
    
    # Combine everything into C++ file
    complete_code = """
#include <Arduino.h>
#include <math.h>

{0}
{1}

{2}

{3}
""".format(activity_map, feature_extraction, '\n\n'.join(tree_functions), detection_function)
    
    # Write to file
    model_code_path = os.path.join(output_dir, 'fall_detection_model.cpp')
    with open(model_code_path, 'w') as f:
        f.write(complete_code)
    
    # Generate header file
    header_code = """
#ifndef FALL_DETECTION_MODEL_H
#define FALL_DETECTION_MODEL_H

#include <Arduino.h>

void extract_features(float accel_x[], float accel_y[], float accel_z[], 
                     int window_size, float features[]);
"""

    # Add activity detection for multi-class
    if is_multiclass:
        header_code += "int detect_activity(float features[]);\n\n"
    
    # Always include fall detection
    header_code += "bool detect_fall(float features[]);\n\n"
    header_code += "#endif // FALL_DETECTION_MODEL_H\n"
    
    header_path = os.path.join(output_dir, 'fall_detection_model.h')
    with open(header_path, 'w') as f:
        f.write(header_code)
    
    print(f"Saved ESP32 model code to {model_code_path} and {header_path}")
    
    # Generate activity detection example for multi-class
    if is_multiclass and label_encoder is not None:
        example_code = """
// Example of using the activity detection function
void displayActivity(int activity) {
  switch(activity) {
"""
        for i, activity in enumerate(label_encoder.classes_):
            example_code += f"    case {i}: Serial.println(\"{activity}\"); break;\n"
        
        example_code += """    default: Serial.println("Unknown activity");
  }
}

void loop() {
  // ... existing code to read sensors and extract features ...
  
  // Detect activity
  int activity = detect_activity(features);
  
  // Display the detected activity
  Serial.print("Detected activity: ");
  displayActivity(activity);
  
  // Also check if it's a fall
  if (detect_fall(features)) {
    Serial.println("FALL DETECTED!");
    // ... handle fall detection ...
  }
  
  delay(1000);  // Adjust as needed
}
"""
        
        example_path = os.path.join(output_dir, 'activity_detection_example.txt')
        with open(example_path, 'w') as f:
            f.write(example_code)
        
        print(f"Saved activity detection example to {example_path}")

if __name__ == "__main__":
    # Define activities to exclude
    exclude_activities = ['walkFall']  # Loại bỏ walkFall
    
    # Load feature dataset, excluding specified activities
    feature_data = load_feature_dataset(exclude_activities=exclude_activities)
    
    # Decide which classification to use: 'ActivityType' for multi-class or 'IsFall' for binary
    target_column = 'ActivityType'  # Change to 'IsFall' for binary classification
    
    # Prepare data for training
    X_train, X_test, y_train, y_test, feature_names, label_encoder = prepare_data(
        feature_data, target_column=target_column)
    
    # Determine number of classes
    n_classes = len(np.unique(y_train))
    print(f"\nPerforming {n_classes}-class classification")
    
    # Train Random Forest model
    rf_model = train_random_forest(X_train, y_train, X_test, y_test, n_classes)
    
    # Evaluate the model
    metrics = evaluate_model(rf_model, X_test, y_test, label_encoder)
    
    # Save model
    save_model(rf_model, label_encoder)
    
    # Convert model to C code for ESP32
    convert_model_for_esp32(rf_model, feature_names, label_encoder)
    
    print("Model training complete! Check results in d:/Downloads/AI_ML/models/results/")