"""
Minimal RP2040 test firmware for debugging
"""
import sys
import time

def main():
    print("RP2040 Test Controller ready")
    
    while True:
        try:
            line = sys.stdin.readline()
            if line:
                line = line.strip()
                if line:
                    cmd = line.upper()
                    if cmd.startswith('<') and cmd.endswith('>'):
                        cmd = cmd[1:-1]
                    
                    if cmd == "PING":
                        print("PONG")
                    elif cmd == "READ":
                        # Always return test sensor data
                        print("<0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000000,0.000000,0.000000,1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,9.810000,2.000,1.500,1.800>")
                    else:
                        print("OK")
                    
                    sys.stdout.flush()
        except:
            pass

if __name__ == "__main__":
    main()
