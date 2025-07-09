import socket # Bluetooth communication module

# Function to connect to the Bluetooth module
def connessione_bluetooth(HC05_MAC,PORT):
    sock=None
    try:
        print(f"Connessione a {HC05_MAC} sulla porta {PORT}...")

        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM) # Create a Bluetooth RFCOMM socket

        # Connect to the specified MAC address and port
        sock.connect((HC05_MAC, PORT))
        print("Connessione riuscita!")

        return sock # Return the connected socket
    except Exception as e:
        print(f"Errore nella connessione Bluetooth: {e}")
        return None

# Function to clear the Bluetooth buffer by reading all available data
def flush_buffer(sock):
    try:
        sock.settimeout(0.1)  # Set a short timeout to avoid blocking
        while True:
            if not sock.recv(1024): # Try to receive up to 1024 bytes
                break
    except:
        pass # Ignore any errors silently
    finally:
        sock.settimeout(None)  # Restore default blocking behavior

# Function to send a message to the Bluetooth device
def invia_messaggio(msg,sock):
    try:
        sock.send(msg.encode()) # Convert the message to bytes and send it
        print(f"Messaggio inviato: {msg}")
    except Exception as e:
        print(f"Errore nell'invio del messaggio: {e}")

# Function to receive a message from the Bluetooth device
def ricevi_messaggio(sock):
    try:
        data = b""  # Initialize an empty bytes object
        while True:
            chunk = sock.recv(1) # Receive one byte at a time
            if not chunk:
                break
            data += chunk # Add the byte to the data buffer
            if chunk == b"\n":  # Stop when newline character is received
                break
        return data.decode().strip() # Decode bytes to string and remove spaces
    except Exception as e:
        print(f"Errore nella ricezione del messaggio: {e}")
        return None

# Function to close the Bluetooth socket
def chiudi_connessione(sock):
    if sock: # Check if the socket is valid
        sock.close()
        print("Connessione chiusa.")
