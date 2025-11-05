
import socket
import time

HOST = "127.0.0.1"
PORT = 8080

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # simple retry until server is ready
    while True:
        try:
            s.connect((HOST, PORT))
            break
        except OSError:
            time.sleep(0.2)

    print(f"[client] connected to {HOST}:{PORT}")
    try:
        while True:
            try:
                line = input()  # blocks; user types here
            except EOFError:
                break

            if line == "quit":
                # send a last line (optional), then close
                try:
                    s.sendall((line + "\n").encode("utf-8"))
                except OSError:
                    pass
                break

            # send line with newline delimiter
            try:
                s.sendall((line + "\n").encode("utf-8"))
            except BrokenPipeError:
                print("[client] server closed the connection.")
                break
            except OSError as e:
                print(f"[client] send error: {e}")
                break
    finally:
        s.close()
        print("[client] closed")

if __name__ == "__main__":
    main()