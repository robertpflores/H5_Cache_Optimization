import serial
import matplotlib.pyplot as plt
import time

# --- CONFIGURATION ---
# Replace with your actual port from 'ls /dev/tty.usbmodem*'
SERIAL_PORT = '/dev/tty.usbmodem1102' 
BAUD_RATE = 115200
TIMEOUT = 5 # Seconds to wait for data

def run_benchmark():
    n_values = []
    std_results = []
    tiled_results = []

    print(f"Connecting to {SERIAL_PORT}...")
    
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
            print("Connected! Waiting for STM32 data...")
            print("Format expected: DATA,N,Std_Cycles,Tiled_Cycles")
            
            # Collect 5 data points (adjust based on how many N sweeps you do)
            while len(n_values) < 5:
                line = ser.readline().decode('utf-8').strip()
                
                if line.startswith("DATA"):
                    parts = line.split(',')
                    # DATA, N, Std, Tiled
                    n = int(parts[1])
                    std = int(parts[2])
                    tiled = int(parts[3])
                    
                    n_values.append(n)
                    std_results.append(std)
                    tiled_results.append(tiled)
                    
                    print(f"Received N={n}: Std={std} cycles, Tiled={tiled} cycles")
                    
    except Exception as e:
        print(f"Error: {e}")
        return

    # --- PLOTTING ---
    plt.figure(figsize=(10, 6))
    
    # Plot raw cycles
    plt.plot(n_values, std_results, 'r-o', label='Standard (Naive)')
    plt.plot(n_values, tiled_results, 'b-s', label='Tiled (Cache-Optimized)')
    
    plt.title('STM32H5 Matrix Multiplication Performance')
    plt.xlabel('Matrix Size (N x N)')
    plt.ylabel('CPU Cycles (DWT->CYCCNT)')
    plt.grid(True, which="both", ls="-", alpha=0.5)
    plt.legend()
    
    # Calculate and show speedup
    for i, n in enumerate(n_values):
        speedup = std_results[i] / tiled_results[i]
        plt.annotate(f"{speedup:.1f}x", (n_values[i], tiled_results[i]), 
                     textcoords="offset points", xytext=(0,10), ha='center')

    plt.show()

if __name__ == "__main__":
    run_benchmark()