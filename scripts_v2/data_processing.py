# ---------------------------------------------------------- #
# DATA MANAGEMENT FUNCTIONS FOR UR ROBOT DEGRADATION TESTING #                            
# BS. 11.04.25                                               #
# ---------------------------------------------------------- #

import csv
import pandas as pd
import matplotlib.pyplot as plt

def save_csv(filename, fields, samples):
    """
    Saves sample data to a CSV file.

    Args:
        filename (str): Base name for the CSV file (without extension).
        fields (list): List of column headers for the CSV.
        samples (dict): Dictionary of Sample objects containing data to be saved.

    Returns:
        str: Path to the generated CSV file.
    """
    csv_file = f"{filename}.csv"  # Construct the full filename with .csv extension
    
    # Open the file in write mode
    with open(csv_file, "w") as csvfile:
        csv_writer = csv.writer(csvfile)  # Create a CSV writer object
        csv_writer.writerow(fields)  # Write the header row
        
        # Write data for each sample if it contains data
        for sample in samples.values():
            if sample.data:
                csv_writer.writerow([sample.id] + sample.data)
    
    return csv_file  # Return the path to the saved CSV file


def plot_graph(csv_file, output_file, hdel, mdel):
    """
    Generates and saves a line graph from a CSV file containing sample data.

    Args:
        csv_file (str): Path to the input CSV file.
        output_file (str): Path to save the generated plot.
        hdel (int): Hours of delay to be displayed in the x-axis label.
        mdel (int): Minutes of delay to be displayed in the x-axis label.
    """
    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(csv_file)
    
    # Extract columns that contain "Average" in their name
    average_columns = [col for col in df.columns if "Average" in col]
    
    # Get unique sample IDs from the "Sample" column
    samples = df["Sample"].unique()
    
    # Extract cycle numbers from the average column names
    cycles = [int(col.split()[1]) for col in average_columns]
    
    # Initialize a dictionary to hold average weights for each sample
    average_weights = {sample: [] for sample in samples}

    # Populate the dictionary with average weight data for each sample
    for sample in samples:
        for col in average_columns:
            average_weights[sample].append(df[df["Sample"] == sample][col].values[0])

    # Configure the plot size
    plt.figure(figsize=(14, 8))
    
    # Plot the data for each sample
    for sample, averages in average_weights.items():
        plt.plot(cycles, averages, marker="o", label=f"Sample {sample}")
    
    # Add title and axis labels
    plt.title("Average Weight per Cycle", fontsize=16)
    plt.xlabel(f"Cycle Number, {hdel} Hr {mdel} Min Delay", fontsize=14)
    plt.ylabel("Average Weight (g)", fontsize=14)
    
    # Customize x and y ticks
    plt.xticks(cycles)
    plt.yticks(fontsize=12)
    
    # Add legend outside the plot
    plt.legend(loc="upper left", bbox_to_anchor=(1, 1), fontsize=12)
    
    # Enable grid for better readability
    plt.grid(True)
    
    # Save the plot as an image file
    plt.savefig(output_file, bbox_inches="tight")
    plt.close()  # Close the plot to free memory


