import pandas as pd
import re
import matplotlib.pyplot as plt
import numpy as np

# Define the input and output file paths
input_file = 'serial_20240810_131554.txt'
output_file = 'heading_movements.csv'

# Define regex patterns to match the desired headings, current headings, heading differences, and auto/obstacle movements
desired_heading_pattern = re.compile(r"Desired Heading:\s([\d\.]+)")
current_heading_pattern = re.compile(r"Current Heading:\s([\d\.]+)")
heading_difference_pattern = re.compile(r"Heading Difference:\s([-\d\.]+)")
movement_pattern = re.compile(r"(Auto:[\w\s]+|No Obsticle:[\w\s]+)")
tilt_movement = re.compile(r"(Forward|Backward) (left|Right) tilt")
right_success_pattern = re.compile(r"UpdatedRightRate:(\d+\.\d+)") 
left_success_pattern = re.compile(r"UpdatedLeftRate:(\d+\.\d+)")

# Initialize lists to store the extracted data
desired_heading = []
current_heading = []
heading_difference = []
movements = []
tilt = []
left_rates = ['0.5']
right_rates = ['0.5']

# Open the input file and process line by line
with open(input_file, 'r') as file:
    lines = file.readlines()
    
    for line in lines:
        desired_heading_match = desired_heading_pattern.search(line)
        current_heading_match = current_heading_pattern.search(line)
        heading_difference_match = heading_difference_pattern.search(line)
        movement_match = movement_pattern.search(line)
        tilt_match = tilt_movement.search(line)
        left_match = left_success_pattern.search(line)
        right_match = right_success_pattern.search(line)
        
        if desired_heading_match:
            desired_heading.append(desired_heading_match.group(1))
        if current_heading_match:
            current_heading.append(current_heading_match.group(1))
        if left_match:
            left_rates.append(left_match.group(1))
        if right_match:
            right_rates.append(right_match.group(1))
        if heading_difference_match:
            heading_difference.append(heading_difference_match.group(1))
        if movement_match:
            movements.append(movement_match.group(0).strip())
        if tilt_match:
            movements.append(tilt_match.group(0).strip())

# Find the maximum length of all lists
max_length = max(len(desired_heading), len(current_heading), len(heading_difference), len(movements))

# Pad the lists with None to make them the same length
desired_heading.extend([None] * (max_length - len(desired_heading)))
current_heading.extend([None] * (max_length - len(current_heading)))
heading_difference.extend([None] * (max_length - len(heading_difference)))
movements.extend([None] * (max_length - len(movements)))
left_rates.extend([None] * (max_length - len(left_rates)))
right_rates.extend([None] * (max_length - len(right_rates)))

# Create a DataFrame from the lists
df = pd.DataFrame({
    'Desired Heading': desired_heading,
    'Current Heading': current_heading,
    'Heading Difference': heading_difference,
    'Movements': movements,
    'left_weightage': left_rates,
    'right_weightage': right_rates
})

# Save the DataFrame to a CSV file
df.to_csv(output_file, index=False)

print(f"Data successfully saved to {output_file}")

print(df['Movements'].unique())

############################################ path visuals

# Define the parameters
turn_angle_left = 0.0005  # Degrees to turn left
turn_angle_right = 0.0006  # Degrees to turn right
tilt_angle_left = 0.00  # Degrees to tilt left (was right)
tilt_angle_right = 0.000001  # Degrees to tilt right (was left)
length_forward = 2  # Length covered in forward movement
length_tilt = 0.205  # Length covered in tilt movement
length_backwards = 2  # Length covered in backward movement
turn_length = 0.01  # Length covered during turns
tilt_turn_length = 0.5  # Length covered during tilt turns
turn_angle_forward = 0.000101

# Initialize position and direction
x, y = 0, 0
angle = 0  # Angle in degrees
positions = [(x, y)]
colors = []
labels = []
backward_positions = []  # List to store positions for big dots
color_map = {
    'Auto:Turning Forward': 'blue',
    'Auto:Turning Left': 'green',
    'No Obsticle: Moving straight': 'blue',
    'Auto:Turning Right': 'red',
    'Forward Right tilt': 'orange',  # Tilt right as per new definition
    'Forward left tilt': 'purple',   # Tilt left as per new definition
    'Auto:Stop': 'black',
    'Auto:Backwards': 'gray'
}

# Convert angle to radians
def angle_to_radians(angle):
    return np.deg2rad(angle)

for instruction in movements:
    if instruction == 'Auto:Turning Forward' or instruction == 'No Obsticle: Moving straight':
        angle += turn_angle_forward
        dx = length_forward * np.cos(angle_to_radians(angle))
        dy = length_forward * np.sin(angle_to_radians(angle))
        x += dx
        y += dy
        positions.append((x, y))
        colors.append(color_map[instruction])
        labels.append(instruction)
        
    elif instruction == 'Auto:Turning Left':
        # Turn left and move forward
        angle += turn_angle_left
        dx = turn_length * np.cos(angle_to_radians(angle))
        dy = turn_length * np.sin(angle_to_radians(angle))
        x += dx
        y += dy
        positions.append((x, y))
        colors.append(color_map[instruction])
        labels.append(instruction)
        
    elif instruction == 'Auto:Turning Right':
        # Turn right and move forward
        angle -= turn_angle_right
        dx = turn_length * np.cos(angle_to_radians(angle))
        dy = turn_length * np.sin(angle_to_radians(angle))
        x += dx
        y += dy
        positions.append((x, y))
        colors.append(color_map[instruction])
        labels.append(instruction)
        
    elif instruction == 'Forward Right tilt':
        # Apply tilt to the right relative to current angle (now correct)
        tilt_angle = angle + tilt_angle_right
        x += length_tilt * np.cos(angle_to_radians(tilt_angle))
        y += length_tilt * np.sin(angle_to_radians(tilt_angle))
        positions.append((x, y))
        colors.append(color_map[instruction])
        labels.append(instruction)
        
    elif instruction == 'Forward left tilt':
        # Apply tilt to the left relative to current angle (now correct)
        tilt_angle = angle - tilt_angle_left
        x += length_tilt * np.cos(angle_to_radians(tilt_angle))
        y += length_tilt * np.sin(angle_to_radians(tilt_angle))
        positions.append((x, y))
        colors.append(color_map[instruction])
        labels.append(instruction)
        
    elif instruction == 'Auto:Stop':
        colors.append(color_map[instruction])
        labels.append(instruction)
        
    elif instruction == 'Auto:Backwards':
        # Mark with a big black dot for backwards movement
        dx = length_backwards * np.cos(angle_to_radians(angle))
        dy = length_backwards * np.sin(angle_to_radians(angle))
        x -= dx
        y -= dy
        positions.append((x, y))
        colors.append(color_map[instruction])
        labels.append(instruction)
        backward_positions.append((x, y))  # Store position for big dot

# Extract x and y coordinates
x_coords, y_coords = zip(*positions)

# Create the plot
plt.figure(figsize=(10, 6))
unique_labels = list(set(labels))  # Get unique labels for the legend

for i in range(len(x_coords) - 1):
    plt.plot([x_coords[i], x_coords[i + 1]], [y_coords[i], y_coords[i + 1]], color=colors[i], marker='o', label=labels[i])

# Plot big black dots for backwards movements
backward_x, backward_y = zip(*backward_positions)
plt.plot(backward_x, backward_y, 'ko', markersize=10, label='Auto:Backwards')

# Create legend
handles = [plt.Line2D([0], [0], color=color_map[label], lw=2) for label in unique_labels]
handles.append(plt.Line2D([0], [0], color='black', marker='o', markersize=10, linestyle='None', label='Auto:Backwards'))
plt.legend(handles, unique_labels + ['Auto:Backwards'])

plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Path Following Instructions')
plt.grid(True)
plt.ylim(-5, 10)
plt.show()



# Direction Visualisation

# Convert data list to a NumPy array with numeric type
data = np.array(current_heading, dtype=float)
desired = np.array(desired_heading, dtype=float)

# Remove NaNs and Infs from both data arrays
data_clean = data[np.isfinite(data)]
desired_clean = desired[np.isfinite(desired)]

# Debug: Print the cleaned data
print("Cleaned Data:", data_clean)
print("Cleaned Desired Data:", desired_clean)
print("Any NaNs after cleaning:", np.isnan(data_clean).any())
print("Any Infs after cleaning:", np.isinf(data_clean).any())
print("Any NaNs in desired after cleaning:", np.isnan(desired_clean).any())
print("Any Infs in desired after cleaning:", np.isinf(desired_clean).any())

# Number of data points
num_data = len(data_clean)
num_desired = len(desired_clean)

# Angles for each data point (360 degrees divided by number of points)
angles = np.linspace(0, 2 * np.pi, num_data, endpoint=False)
desired_angles = np.linspace(0, 2 * np.pi, num_desired, endpoint=False)

# Create the figure and axis objects for both plots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))

# Polar plot
ax1 = plt.subplot(2, 1, 1, polar=True)
for angle, value in zip(angles, data_clean):
    ax1.plot([angle, angle], [0, value], color='blue', linestyle='-', linewidth=1)
for desired_angle, desired_value in zip(desired_angles, desired_clean):
    ax1.plot([desired_angle, desired_angle], [0, value], color='red', linestyle='-', linewidth=0.1)
ax1.set_theta_zero_location('N')
ax1.set_theta_direction(-1)
if len(data_clean) > 0:
    max_data_clean = np.max(data_clean)
    ax1.set_ylim(0, max_data_clean + 10)
ax1.legend()

# Bar graph
x_positions = np.arange(num_data)  # X positions for each bar
ax2.bar(x_positions, data_clean, color='blue')
# Horizontal lines for desired_heading
for value in desired_clean:
    ax2.plot([0, num_data-1], [value, value], color='red', linestyle='-', linewidth=0.02,alpha=0.2)
ax2.set_xlabel('Data Point Index')
ax2.set_ylabel('Value of vehicle\'s heading angle along the path')
ax2.set_ylim(0, 360)
ax2.set_title('Bar Graph of heading angle with Desired Headings')
ax2.legend()

# Show the plots
plt.tight_layout()
plt.legend()
plt.show()

#### Turning Rate Plots ###
# Convert to NumPy arrays and filter out None values
left_array = np.array(left_rates, dtype=float)
left_clean = left_array[~np.isnan(left_array)]

right_array = np.array(right_rates, dtype=float)
right_clean = right_array[~np.isnan(right_array)]

# Create a figure and axis
plt.figure(figsize=(10, 6))

# Plot Left Turn Weightage
plt.plot(left_clean, marker='o', linestyle='-', color='b', label='Left Turn Weightage')

# Plot Right Turn Weightage
plt.plot(right_clean, marker='o', linestyle='--', color='r', label='Right Turn Weightage')

# Adding labels and title
plt.xlabel('Index')
plt.ylabel('Weightage Value')
plt.title('Left and Right Turn Weightage Along the Path')
plt.legend()
plt.grid(True)

# Show plot
plt.show()

left=[]
right=[]

index = 0
left_count = 0
right_count = 0
for m in movements:
    if movements[index]=='Auto:Backwards':
        print(movements[index])
        if movements[index+1]=='Auto:Turning Left':
            left_count+=1
            # print(movements[index+1])
        elif movements[index+1]=='Auto:Turning Right':
            right_count+=1
            # print(right_count)
    index+=1

print(left_count,right_count)