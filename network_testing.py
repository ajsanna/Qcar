import subprocess
import time
def get_wifi_networks():
   """Gets the list of available Wi-Fi networks."""

   command = "nmcli device wifi list"
   output = subprocess.check_output(command, shell=True)
   return output
def formatter(input):
    decoded_data = input.decode('utf-8')

    # Split the data into lines
    lines = decoded_data.splitlines()

    # Print each line, formatted for better readability
    for line in lines:
        print(line.strip())

def main():
    networks = get_wifi_networks()
    formatter(networks)
if __name__ == "__main__":
    while(True):
        main()
        print("\n" + "-" * 50)
        time.sleep(3)
    
