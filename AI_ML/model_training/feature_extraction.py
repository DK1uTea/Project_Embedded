import pandas as pd
import numpy as np
import os
import glob
from tqdm import tqdm
from scipy import stats
import matplotlib.pyplot as plt
from scipy.fft import fft
import seaborn as sns

def load_preprocessed_data(file_path='d:/Downloads/AI_ML/data/processed/preprocessed_data.csv'):
    """
    Load preprocessed data
    
    Args:
        file_path: Path to the preprocessed CSV file
    
    Returns:
        DataFrame with preprocessed data
    """
    print(f"Loading preprocessed data from {file_path}")
    return pd.read_csv(file_path)

def create_windows(df, window_size=50, overlap=0.5):
    """
    Segment the data into overlapping windows
    
    Args:
        df: DataFrame with preprocessed data
        window_size: Number of samples in each window
        overlap: Fraction of overlap between consecutive windows
    
    Returns:
        List of window DataFrames and corresponding labels
    """
    windows = []
    labels = []
    activity_types = []
    source_files = []
    
    # Calculate step size
    step = int(window_size * (1 - overlap))
    
    # Group by source file to avoid creating windows across different recordings
    grouped = df.groupby(['SourceFile'])
    
    print(f"Creating windows with size {window_size} and overlap {overlap}...")
    for name, group in tqdm(grouped):
        # Skip if group has fewer samples than window size
        if len(group) < window_size:
            continue
        
        # Sort by timestamp to ensure chronological order
        group = group.sort_values('Timestamp')
        
        # Get fall label and activity type
        is_fall = group['IsFall'].iloc[0]
        activity = group['ActivityType'].iloc[0]
        
        # Create windows with the specified overlap
        for i in range(0, len(group) - window_size + 1, step):
            window = group.iloc[i:i+window_size]
            windows.append(window)
            labels.append(is_fall)
            activity_types.append(activity)
            source_files.append(name)
    
    print(f"Created {len(windows)} windows")
    return windows, labels, activity_types, source_files

def extract_features(window):
    """
    Extract time and frequency domain features from a window of sensor data
    
    Args:
        window: DataFrame containing a window of sensor data
    
    Returns:
        Dictionary of extracted features
    """
    features = {}
    
    # Extract time domain features for each acceleration axis and magnitude
    for signal in ['AccelerationX', 'AccelerationY', 'AccelerationZ', 'AccMagnitude']:
        # Basic statistical features
        features[f'{signal}_mean'] = window[signal].mean()
        features[f'{signal}_std'] = window[signal].std()
        features[f'{signal}_min'] = window[signal].min()
        features[f'{signal}_max'] = window[signal].max()
        features[f'{signal}_range'] = window[signal].max() - window[signal].min()
        features[f'{signal}_median'] = window[signal].median()
        
        # Percentiles
        features[f'{signal}_25percentile'] = window[signal].quantile(0.25)
        features[f'{signal}_75percentile'] = window[signal].quantile(0.75)
        features[f'{signal}_iqr'] = features[f'{signal}_75percentile'] - features[f'{signal}_25percentile']
        
        # Higher order statistics
        features[f'{signal}_skewness'] = stats.skew(window[signal])
        features[f'{signal}_kurtosis'] = stats.kurtosis(window[signal])
        
        # Energy and power
        features[f'{signal}_energy'] = np.sum(window[signal]**2) / len(window)
        
        # Zero crossings (sign changes)
        features[f'{signal}_zero_crossings'] = np.sum(np.diff(np.signbit(window[signal])))
        
        # Mean absolute deviation
        features[f'{signal}_mad'] = np.mean(np.abs(window[signal] - window[signal].mean()))
        
        # Peak features
        if signal == 'AccMagnitude':
            # Calculate peak-to-peak amplitude
            features[f'{signal}_p2p'] = features[f'{signal}_max'] - features[f'{signal}_min']
            
            # Find peaks and calculate features related to them
            # Higher peaks might indicate a fall
            peaks = window[window[signal] > features[f'{signal}_mean'] + features[f'{signal}_std']][signal]
            if len(peaks) > 0:
                features[f'{signal}_peak_mean'] = peaks.mean()
                features[f'{signal}_peak_max'] = peaks.max()
                features[f'{signal}_peak_count'] = len(peaks)
            else:
                features[f'{signal}_peak_mean'] = features[f'{signal}_mean']
                features[f'{signal}_peak_max'] = features[f'{signal}_max']
                features[f'{signal}_peak_count'] = 0
    
    # Extract jerk features if available
    if 'JerkX' in window.columns:
        for signal in ['JerkX', 'JerkY', 'JerkZ', 'JerkMagnitude']:
            features[f'{signal}_mean'] = window[signal].mean()
            features[f'{signal}_std'] = window[signal].std()
            features[f'{signal}_max'] = window[signal].max()
            
            # Jerk energy - useful for detecting sudden movements like falls
            features[f'{signal}_energy'] = np.sum(window[signal]**2) / len(window)
    
    # Correlation between axes
    features['correlation_xy'] = window['AccelerationX'].corr(window['AccelerationY'])
    features['correlation_xz'] = window['AccelerationX'].corr(window['AccelerationZ'])
    features['correlation_yz'] = window['AccelerationY'].corr(window['AccelerationZ'])
    
    # Frequency domain features
    for signal in ['AccelerationX', 'AccelerationY', 'AccelerationZ', 'AccMagnitude']:
        # Apply FFT
        fft_values = fft(window[signal].values)
        fft_magnitude = np.abs(fft_values)[:len(window)//2]
        
        # Frequency domain features
        features[f'{signal}_fft_mean'] = np.mean(fft_magnitude)
        features[f'{signal}_fft_std'] = np.std(fft_magnitude)
        features[f'{signal}_fft_max'] = np.max(fft_magnitude)
        
        # Spectral energy
        features[f'{signal}_spectral_energy'] = np.sum(fft_magnitude**2) / len(fft_magnitude)
    
    return features

def create_feature_dataset(windows, labels, activity_types, source_files):
    """
    Extract features from all windows and create a dataset
    
    Args:
        windows: List of window DataFrames
        labels: List of labels for each window
        activity_types: List of activity types for each window
        source_files: List of source file names for each window
    
    Returns:
        DataFrame with extracted features and labels
    """
    feature_rows = []
    
    print("Extracting features from windows...")
    for i, window in enumerate(tqdm(windows)):
        # Extract features
        features = extract_features(window)
        
        # Add metadata
        features['IsFall'] = labels[i]
        features['ActivityType'] = activity_types[i]
        features['SourceFile'] = source_files[i]
        
        # Add to list
        feature_rows.append(features)
    
    # Create DataFrame
    feature_df = pd.DataFrame(feature_rows)
    print(f"Created feature dataset with {len(feature_df)} samples and {feature_df.shape[1]} features")
    
    return feature_df

def analyze_features(feature_df, output_dir='d:/Downloads/AI_ML/data/features'):
    """
    Analyze and visualize the extracted features
    
    Args:
        feature_df: DataFrame with extracted features
        output_dir: Directory to save visualizations
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # Calculate feature importance using correlation with fall label
    numeric_features = feature_df.select_dtypes(include=[np.number]).columns.tolist()
    numeric_features.remove('IsFall')  # Remove the target variable
    
    # Calculate absolute correlation with IsFall
    correlations = []
    for feature in numeric_features:
        corr = abs(feature_df[feature].corr(feature_df['IsFall']))
        correlations.append((feature, corr))
    
    # Sort by correlation and get top features
    top_features = sorted(correlations, key=lambda x: x[1], reverse=True)[:20]
    
    # Create correlation bar chart
    plt.figure(figsize=(12, 8))
    features = [x[0] for x in top_features]
    corrs = [x[1] for x in top_features]
    
    plt.barh(range(len(features)), corrs, align='center')
    plt.yticks(range(len(features)), features)
    plt.xlabel('Absolute Correlation with Fall')
    plt.title('Top 20 Features by Correlation with Fall')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'feature_importance.png'))
    
    # Visualize distributions of top 5 features
    top_5_features = features[:5]
    plt.figure(figsize=(15, 12))
    
    for i, feature in enumerate(top_5_features):
        plt.subplot(3, 2, i+1)
        sns.histplot(data=feature_df, x=feature, hue='IsFall', kde=True, bins=30)
        plt.title(f'Distribution of {feature}')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'top_features_distribution.png'))
    
    # Correlation matrix of top features
    plt.figure(figsize=(12, 10))
    corr_matrix = feature_df[top_5_features + ['IsFall']].corr()
    sns.heatmap(corr_matrix, annot=True, cmap='coolwarm')
    plt.title('Correlation Matrix of Top Features')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'correlation_matrix.png'))
    
    print(f"Saved feature analysis visualizations to {output_dir}")
    
    return top_features

def save_feature_dataset(feature_df, output_dir='d:/Downloads/AI_ML/data/features'):
    """
    Save the feature dataset
    
    Args:
        feature_df: DataFrame with extracted features
        output_dir: Directory to save the dataset
    """
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, 'extracted_features.csv')
    feature_df.to_csv(output_file, index=False)
    print(f"Saved feature dataset to {output_file}")

if __name__ == "__main__":
    # Load preprocessed data
    preprocessed_data = load_preprocessed_data()
    
    # Create windows
    windows, labels, activity_types, source_files = create_windows(preprocessed_data)
    
    # Extract features and create dataset
    feature_dataset = create_feature_dataset(windows, labels, activity_types, source_files)
    
    # Analyze features
    top_features = analyze_features(feature_dataset)
    print("Top 10 most important features:")
    for i, (feature, corr) in enumerate(top_features[:10]):
        print(f"{i+1}. {feature}: {corr:.4f}")
    
    # Save feature dataset
    save_feature_dataset(feature_dataset)