import os

def rename_files(directory):
    for root, dirs, files in os.walk(directory):
        for filename in files:
            if filename.endswith('.txt'):
                # Split the filename by underscores
                parts = filename.split('_')
                
                # Find the index of 'png' in the parts
                png_index = next((i for i, part in enumerate(parts) if 'png' in part), None)
                
                if png_index is not None:
                    # Reconstruct the new filename
                    new_filename = '_'.join(parts[:png_index]) + '.txt'
                    
                    # Rename the file
                    old_path = os.path.join(root, filename)
                    new_path = os.path.join(root, new_filename)
                    os.rename(old_path, new_path)
                    print(f"Renamed: {filename} -> {new_filename}")

# Path to the SegmentationLabels folder
base_path = '.'

# Iterate through terrain folders
for i in range(1, 14):  # Assuming terrain01 to terrain13
    terrain_folder = f'terrain{i:02d}/labels'
    full_path = os.path.join(base_path, terrain_folder)
    if os.path.exists(full_path):
        print(f"Processing {terrain_folder}...")
        rename_files(full_path)

print("File renaming completed.")
