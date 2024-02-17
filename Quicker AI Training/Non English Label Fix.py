import os
import fileinput

def replace_commas_with_periods_in_files(directory):
    # Get a list of all .txt files in the directory
    txt_files = [f for f in os.listdir(directory) if f.endswith('.txt')]
    
    # Iterate through each .txt file
    for txt_file in txt_files:
        file_path = os.path.join(directory, txt_file)
        
        # Open the file for in-place editing
        with fileinput.FileInput(file_path, inplace=True) as file:
            # Iterate through each line in the file
            for line in file:
                # Replace commas with periods
                modified_line = line.replace(',', '.')
                # Write the modified line to the file
                print(modified_line, end='')

# Specify the directory containing the .txt files
directory_path = '/path/to/your/directory'

# Call the function to replace commas with periods in all .txt files in the directory
replace_commas_with_periods_in_files(directory_path)
