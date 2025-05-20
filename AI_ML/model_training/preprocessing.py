import pandas as pd
import numpy as np
import os
import glob
from tqdm import tqdm
import matplotlib.pyplot as plt

def load_data(base_dir=None):
    """
    Load data from all activity folders and label them as fall or non-fall
    
    Args:
        base_dir: Base directory containing activity data folders
    
    Returns:
        DataFrame with loaded data and labels
    """
    # Try to automatically determine the correct base directory
    if base_dir is None:
        # Try several possible paths
        possible_paths = [
            'd:/Downloads/AI_ML/data',
            './data',
            '../data',
            'data'
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                base_dir = path
                print(f"Automatically detected data directory: {os.path.abspath(base_dir)}")
                break
    
    if base_dir is None or not os.path.exists(base_dir):
        raise ValueError(f"Data directory not found. Please specify the correct path.")
        
    # Define activity types and whether they contain falls
    activity_types = {
        'downSit': False,  # Sitting down (not a fall)
        'freeFall': True,   # Free fall
        'runFall': True,    # Running and falling
        'runSit': False,    # Running and sitting (not a fall)
        'walkFall': True,   # Walking and falling
        'walkSit': False    # Walking and sitting (not a fall)
    }
    
    all_dfs = []
    
    for activity, is_fall in activity_types.items():
        print(f"Loading {activity} data...")
        folder_path = os.path.join(base_dir, activity)
        
        # Check if directory exists
        if not os.path.exists(folder_path):
            print(f"WARNING: Folder {folder_path} does not exist. Skipping.")
            continue
            
        # Try to find CSV files with different patterns
        csv_files = []
        csv_files.extend(glob.glob(os.path.join(folder_path, '*.csv')))
        csv_files.extend(glob.glob(os.path.join(folder_path, '*', '*.csv')))  # Check subdirectories too
        
        # Check if any CSV files were found
        if not csv_files:
            print(f"WARNING: No CSV files found in {folder_path}. Skipping.")
            continue
        
        print(f"Found {len(csv_files)} CSV files in {folder_path}")
            
        for file in tqdm(csv_files):
            try:
                # First try with semicolon separator
                try:
                    df = pd.read_csv(file, sep=';', header=None, on_bad_lines='skip')
                except:
                    # If that fails, try with comma separator
                    try:
                        df = pd.read_csv(file, sep=',', header=None, on_bad_lines='skip')
                    except:
                        # Last resort, try with automatic separator detection
                        df = pd.read_csv(file, sep=None, engine='python', header=None)
                
                # Validate the dataframe has at least 6 columns
                if df.shape[1] < 6:
                    print(f"WARNING: File {file} has only {df.shape[1]} columns. Expected at least 6. Skipping.")
                    continue
                
                # Add proper column names for the first 6 columns
                column_names = ['DateString', 'Timestamp', 'Orientation', 
                               'AccelerationX', 'AccelerationY', 'AccelerationZ']
                
                # Rename only the first 6 columns
                df = df.iloc[:, :6]
                df.columns = column_names
                
                # Clean the file lines by removing any rows with non-numeric acceleration values
                for col in ['AccelerationX', 'AccelerationY', 'AccelerationZ']:
                    df[col] = pd.to_numeric(df[col], errors='coerce')
                
                # Remove rows with NaN values
                df = df.dropna()
                
                # Skip if no rows remain
                if len(df) == 0:
                    print(f"WARNING: File {file} has no valid data after cleaning. Skipping.")
                    continue
                
                # Add activity type and fall label
                df['ActivityType'] = activity
                df['IsFall'] = 1 if is_fall else 0
                df['SourceFile'] = os.path.basename(file)
                
                all_dfs.append(df)
                print(f"Successfully loaded {len(df)} rows from {file}")
            except Exception as e:
                print(f"Error processing {file}: {e}")
    
    # Check if any dataframes were created
    if not all_dfs:
        raise ValueError("No data was loaded. Please check your data directory structure.")
        
    # Combine all data
    combined_df = pd.concat(all_dfs, ignore_index=True)
    print(f"Loaded {len(combined_df)} samples from {len(all_dfs)} files")
    
    return combined_df

def preprocess_data(df):
    """
    Preprocess the raw data by:
    1. Converting timestamp to numeric
    2. Calculating acceleration magnitude
    3. Removing outliers
    
    Args:
        df: DataFrame with raw data
    
    Returns:
        Preprocessed DataFrame
    """
    # Make a copy to avoid modifying the original
    df_processed = df.copy()
    
    # Ensure timestamp is numeric
    if df_processed['Timestamp'].dtype != 'float64':
        df_processed['Timestamp'] = pd.to_numeric(df_processed['Timestamp'])
    
    # Calculate acceleration magnitude (total g-force)
    df_processed['AccMagnitude'] = np.sqrt(
        df_processed['AccelerationX']**2 + 
        df_processed['AccelerationY']**2 + 
        df_processed['AccelerationZ']**2
    )
    
    # Add jerk (rate of change of acceleration)
    for axis in ['X', 'Y', 'Z']:
        col = f'Acceleration{axis}'
        jerk_col = f'Jerk{axis}'
        # Group by file to avoid calculating jerk between different recordings
        df_processed[jerk_col] = df_processed.groupby('SourceFile')[col].diff() / df_processed.groupby('SourceFile')['Timestamp'].diff()
    
    # Calculate jerk magnitude
    df_processed['JerkMagnitude'] = np.sqrt(
        df_processed['JerkX']**2 + 
        df_processed['JerkY']**2 + 
        df_processed['JerkZ']**2
    )
    
    # Remove outliers - accelerations that are physically impossible (e.g., > 20g)
    # Normal falls might generate up to 10-15g
    df_processed = df_processed[df_processed['AccMagnitude'] < 20]
    
    # Handle NaN values from the diff operation for jerk calculation
    df_processed = df_processed.fillna(method='bfill')
    
    print(f"Preprocessed data shape: {df_processed.shape}")
    
    return df_processed

def visualize_data(df, output_dir='d:/Downloads/AI_ML/data/visualizations'):
    """
    Create visualizations of the data
    
    Args:
        df: Preprocessed DataFrame
        output_dir: Directory to save visualizations
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # Visualize acceleration distribution
    plt.figure(figsize=(10, 6))
    plt.hist(df[df['IsFall'] == 1]['AccMagnitude'], bins=50, alpha=0.5, label='Fall')
    plt.hist(df[df['IsFall'] == 0]['AccMagnitude'], bins=50, alpha=0.5, label='Non-Fall')
    plt.xlabel('Acceleration Magnitude (g)')
    plt.ylabel('Frequency')
    plt.title('Distribution of Acceleration Magnitude')
    plt.legend()
    plt.savefig(os.path.join(output_dir, 'acc_magnitude_distribution.png'))
    
    # Box plot of acceleration by activity type
    plt.figure(figsize=(12, 6))
    df.boxplot(column='AccMagnitude', by='ActivityType')
    plt.title('Acceleration Magnitude by Activity Type')
    plt.suptitle('')  # Remove default title
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'acc_by_activity.png'))
    
    # Example of fall vs non-fall acceleration pattern
    # Make sure we have enough data to visualize
    fall_samples = df[df['IsFall'] == 1].groupby('SourceFile').filter(lambda x: len(x) > 100)
    nonfall_samples = df[df['IsFall'] == 0].groupby('SourceFile').filter(lambda x: len(x) > 100)
    
    if len(fall_samples) > 0 and len(nonfall_samples) > 0:
        # Select one random sample from each category if data is available
        fall_sample = fall_samples.groupby('SourceFile').head(200).sample(1)
        fall_file = fall_sample['SourceFile'].iloc[0]
        fall_data = df[df['SourceFile'] == fall_file].head(200)
        
        nonfall_sample = nonfall_samples.groupby('SourceFile').head(200).sample(1)
        nonfall_file = nonfall_sample['SourceFile'].iloc[0]
        nonfall_data = df[df['SourceFile'] == nonfall_file].head(200)
        
        # Plot fall data
        plt.figure(figsize=(12, 10))
        
        plt.subplot(2, 1, 1)
        plt.plot(fall_data['Timestamp'] - fall_data['Timestamp'].iloc[0], fall_data['AccelerationX'], label='X')
        plt.plot(fall_data['Timestamp'] - fall_data['Timestamp'].iloc[0], fall_data['AccelerationY'], label='Y')
        plt.plot(fall_data['Timestamp'] - fall_data['Timestamp'].iloc[0], fall_data['AccelerationZ'], label='Z')
        plt.plot(fall_data['Timestamp'] - fall_data['Timestamp'].iloc[0], fall_data['AccMagnitude'], label='Magnitude', linewidth=2)
        plt.title(f'Fall Example: {fall_data["ActivityType"].iloc[0]}')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (g)')
        plt.legend()
        
        plt.subplot(2, 1, 2)
        plt.plot(nonfall_data['Timestamp'] - nonfall_data['Timestamp'].iloc[0], nonfall_data['AccelerationX'], label='X')
        plt.plot(nonfall_data['Timestamp'] - nonfall_data['Timestamp'].iloc[0], nonfall_data['AccelerationY'], label='Y')
        plt.plot(nonfall_data['Timestamp'] - nonfall_data['Timestamp'].iloc[0], nonfall_data['AccelerationZ'], label='Z')
        plt.plot(nonfall_data['Timestamp'] - nonfall_data['Timestamp'].iloc[0], nonfall_data['AccMagnitude'], label='Magnitude', linewidth=2)
        plt.title(f'Non-Fall Example: {nonfall_data["ActivityType"].iloc[0]}')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (g)')
        plt.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'fall_vs_nonfall.png'))
    else:
        print("Not enough data to create fall vs non-fall example visualizations")
    
    print(f"Saved visualizations to {output_dir}")

def save_processed_data(df, output_dir='d:/Downloads/AI_ML/data/processed'):
    """
    Save the processed data
    
    Args:
        df: Processed DataFrame
        output_dir: Directory to save the data
    """
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, 'preprocessed_data.csv')
    df.to_csv(output_file, index=False)
    print(f"Saved preprocessed data to {output_file}")

if __name__ == "__main__":
    try:
        # Load raw data
        print("Step 1: Loading raw data...")
        raw_data = load_data()
        
        # Preprocess data
        print("\nStep 2: Preprocessing data...")
        processed_data = preprocess_data(raw_data)
        
        # Create visualizations
        print("\nStep 3: Creating visualizations...")
        visualize_data(processed_data)
        
        # Save processed data
        print("\nStep 4: Saving processed data...")
        save_processed_data(processed_data)
        
        print("\nPreprocessing completed successfully!")
    except Exception as e:
        print(f"\nERROR: Preprocessing failed: {e}")
        print("\nTroubleshooting tips:")
        print("1. Check that your data directory structure is correct")
        print("2. Ensure CSV files have the correct format (semicolon separated)")
        print("3. Verify file permissions")