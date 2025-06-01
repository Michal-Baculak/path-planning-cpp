input_file = "LogFolder/reginfo.txt"  # Your input file with the YAML-like structure
output_file = "LogFolder/reginfo_parsed.txt"

def parse_file(input_file, output_file):
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        lines = infile.readlines()
        secs = nsecs = x = y = z = None
        for line in lines:
            line = line.strip()
            if line.startswith("secs:"):
                secs = int(line.split(":")[1].strip())
            elif line.startswith("nsecs:"):
                nsecs = int(line.split(":")[1].strip())
            elif line.startswith("x:"):
                x = float(line.split(":")[1].strip())
            elif line.startswith("y:"):
                y = float(line.split(":")[1].strip())
            elif line.startswith("z:"):
                z = float(line.split(":")[1].strip())
            elif line.startswith("---"):
                if secs is not None and nsecs is not None and x is not None and y is not None and z is not None:
                    time = secs + nsecs * 1e-9
                    outfile.write(f"{time} {x} {y} {z}\n")
                # Reset for next block
                secs = nsecs = x = y = z = None
        # Handle last block if no trailing '---'
        if secs is not None and nsecs is not None and x is not None and y is not None and z is not None:
            time = secs + nsecs * 1e-9
            outfile.write(f"{time} {x} {y} {z}\n")

parse_file(input_file, output_file)
