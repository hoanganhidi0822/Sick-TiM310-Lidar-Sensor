def parse_telegram(telegram):
    tokens = telegram.split(' ')
    
    # Ensure that there are enough tokens
    if len(tokens) <= (18 + 8):  # Minimum valid length
        raise ValueError("Insufficient data tokens")
    
    # Extract header and validate
    header = tokens[:18]
    if header[0] != 'sRI':
        raise ValueError("Invalid command type")
    if header[1] != 'E9':
        raise ValueError("Invalid command")
    
    # Extract and validate data sections
    sections = tokens[18:]
    print(sections)
    try:
        if int(sections[0], 16) != 0:  # No encoder data
            raise ValueError("Unexpected encoder data")
        if int(sections[1], 16) != 1:  # Exactly 1 16-bit channel block
            raise ValueError("Unexpected channel block count")
        if sections[2] != 'DIST1':  # Distance data expected
            raise ValueError("Unexpected data type")
        if sections[3] not in ['3F800000', '40000000']:  # Check scale factor
            raise ValueError("Invalid scale factor")
        
        scale_factor = 1 if sections[3] == '3F800000' else 2
        if sections[4] != '00000000':
            raise ValueError("Unexpected value in section 4")
        
        start_angle = int(sections[5], 16) / 10000.0
        angle_step = int(sections[6], 16) / 10000.0
        value_count = int(sections[7], 16)
        
        # Extract distance values and compute angles
        values = list(map(lambda x: int(x, 16) * scale_factor, sections[8:8 + value_count]))
        angles = [start_angle + angle_step * n for n in range(value_count)]
        
        return (values, angles)
    
    except ValueError as e:
        raise ValueError(f"Parsing error: {e}")

def main():
    # Example telegram data (make sure this matches your actual sensor data)
    telegram = 'sRI E9 0 1 DIST1 3F800000 00000000 00010000 00020000 00030000 00040000 00050000 00060000 00070000 00080000 00090000 000A0000 000B0000 000C0000 000D0000 000E0000 000F0000 00100000 00110000 00120000 00130000 00140000 00150000 00160000 00170000 00180000 00190000'
    
    # Parse the telegram data
    try:
        values, angles = parse_telegram(telegram)
        print("Distances (in mm):", values)
        print("Angles (in degrees):", angles)
    except ValueError as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
