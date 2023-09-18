import matplotlib.pyplot as plt

# Define the path to your file
file_path = 'a.txt'

# Initialize a variable to store the last line
last_line = None

# Open the file and read the last line
with open(file_path, 'r') as file:
    lines = file.readlines()
    if lines:
        last_line = lines[-1].strip()  # Remove any leading/trailing whitespace

# Check if a valid last line was found
if last_line is not None:
    # Split the last line by commas
    values = last_line.split(',')
    
    # Initialize an empty list for the data
    data = []
    
    # Attempt to convert each value to a float, handling errors
    for value in values:
        try:
            data.append(float(value))
        except ValueError:
            print(f"Warning: Skipping invalid value '{value}'")
    
    if data:
        # Create a line plot
        plt.plot(data)
        plt.ylim(-0.5, 0.5)  # Set the y-axis range to -0.1 to 0.1
        plt.xlabel('Time')
        plt.ylabel('Error Values')
        plt.title('Error Values Over Time')
        
        # Save the plot to an image file (e.g., PNG)
        plt.savefig('error_values_over_time.png')
        
        # Optionally, display the plot
        # plt.show()
        
        print("Line plot saved as 'error_values_over_time.png'.")
    else:
        print("No valid data found in the last line.")
else:
    print("File is empty or couldn't read the last line.")