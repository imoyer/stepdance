#!/usr/bin/env python3
"""
Extract code snippets from snippets.cpp and convert them to .ino files.

This script parses the snippets.cpp file looking for code blocks marked with
// [SnippetName] and // [SnippetName] tags, then creates individual .ino files
in the lib/examples/snippets/ directory.
"""

import re
import os
import shutil
from pathlib import Path

def extract_snippets(input_file, output_dir):
    """
    Extract snippets from input file and create .ino files.
    
    Args:
        input_file: Path to snippets.cpp file
        output_dir: Directory to output .ino files
    """
    output_path = Path(output_dir)
    
    # Clear the output directory if it exists
    if output_path.exists():
        print(f"Clearing existing directory: {output_path}")
        shutil.rmtree(output_path)
    
    # Create output directory
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Read the input file
    with open(input_file, 'r') as f:
        content = f.read()
    
    # Pattern to match snippet blocks: // [Name] ... // [Name]
    # Using non-greedy match and capturing the content between markers
    pattern = r'//\s*\[(\w+)\]\s*\n(.*?)//\s*\[\1\]'
    
    snippets = re.findall(pattern, content, re.DOTALL)
    
    print(f"Found {len(snippets)} snippets in {input_file}")
    
    for snippet_name, snippet_code in snippets:
        # Clean up the code - remove leading/trailing whitespace
        snippet_code = snippet_code.strip()
        
        # Create filename from snippet name (convert CamelCase to snake_case)
        filename = re.sub(r'(?<!^)(?=[A-Z])', '_', snippet_name).lower()
        
        # Create a folder for this snippet
        snippet_folder = output_path / filename
        snippet_folder.mkdir(parents=True, exist_ok=True)
        
        # Create the .ino file inside the folder
        output_file = snippet_folder / f"{filename}.ino"
        
        # Add stepdance.hpp include if not present
        if '#include "stepdance.hpp"' not in snippet_code:
            snippet_code = '#include "stepdance.hpp"\n\n' + snippet_code
        
        # Write the .ino file
        with open(output_file, 'w') as f:
            f.write(snippet_code)
            f.write('\n')
        
        print(f"  Created: {output_file}")
    
    print(f"\nSuccessfully created {len(snippets)} .ino files in {output_dir}")
    return len(snippets)

if __name__ == "__main__":
    # Get the directory where this script is located
    script_dir = Path(__file__).parent
    
    # Define paths relative to script location
    input_file = script_dir / "doxygen" / "snippets.cpp"
    output_dir = script_dir / "lib" / "examples" / "snippets"
    
    # Check if input file exists
    if not input_file.exists():
        print(f"Error: Input file not found: {input_file}")
        exit(1)
    
    # Extract snippets
    count = extract_snippets(input_file, output_dir)
    
    if count == 0:
        print("Warning: No snippets found. Check the input file format.")
        exit(1)
