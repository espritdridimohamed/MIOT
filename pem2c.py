import sys

def pem_to_c_string(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    for line in lines:
        # Remove any trailing \r\n or \n, then add \n explicitly
        print('"' + line.rstrip('\r\n') + '\\n"')

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python pem2c.py <file.pem>")
        sys.exit(1)
    pem_to_c_string(sys.argv[1])
