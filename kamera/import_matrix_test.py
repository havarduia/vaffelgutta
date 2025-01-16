import os
from create_matrix_from_apriltag import save_apriltag_matrix

def test_save_apriltag_matrix():
    # Specify the output file for the transformation matrix
    output_file = "test_transformation_matrix.txt"
    
    # Call the function
    save_apriltag_matrix(output_file=output_file)
    
    # Check if the file was created and print the result
    if os.path.exists(output_file):
        print(f"Test Passed: Transformation matrix saved to {output_file}")
        # Optional: Print the content of the matrix file
        with open(output_file, "r") as f:
            print("Saved Transformation Matrix:")
            print(f.read())
    else:
        print("Test Failed: Transformation matrix not saved")

# Run the test
test_save_apriltag_matrix()
