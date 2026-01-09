#!/usr/bin/env python3
"""
Command Server - TCP interface for remote commands
"""

import socket
import threading
import json
import sys
sys.path.append('/home/pi/Working Code/Updated FULL Code Standalone')
from config import *


class CommandServer:
    """
    TCP server for receiving commands from laptop controller
    """

    def __init__(self, robot_controller):
        self.robot = robot_controller
        self.server_socket = None
        self.running = False
        self.thread = None
        print(f"[Command Server] Initialized on port {COMMAND_PORT}")

    def start(self):
        """Start server in background thread"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', COMMAND_PORT))
        self.server_socket.listen(5)
        self.server_socket.settimeout(1.0)

        self.running = True
        self.thread = threading.Thread(target=self._server_loop, daemon=True)
        self.thread.start()

        print(f"[Command Server] Listening on port {COMMAND_PORT}")

    def _server_loop(self):
        """Accept and handle connections"""
        while self.running:
            try:
                client_socket, addr = self.server_socket.accept()
                print(f"[Command Server] Client connected: {addr}")

                # Handle client in separate thread
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_socket,),
                    daemon=True
                )
                client_thread.start()

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[Command Server] Error: {e}")

    def _handle_client(self, client_socket):
        """Handle single client connection"""
        try:
            client_socket.settimeout(30.0)
            # Set socket options for better connection handling
            client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

            while self.running:
                try:
                    data = client_socket.recv(1024).decode('utf-8').strip()
                    if not data:
                        break

                    # Process command
                    try:
                        response = self.robot.process_command(data)
                    except Exception as e:
                        response = f"ERROR: Command processing failed - {str(e)}"

                    # Send response with error handling for broken pipe
                    try:
                        client_socket.sendall((response + '\n').encode('utf-8'))
                    except BrokenPipeError:
                        print(f"[Command Server] Client disconnected (broken pipe)")
                        break
                    except ConnectionResetError:
                        print(f"[Command Server] Client connection reset")
                        break

                except socket.timeout:
                    continue  # Keep waiting for data
                except (BrokenPipeError, ConnectionResetError):
                    break

        except socket.timeout:
            pass
        except Exception as e:
            # Don't print broken pipe errors as they're expected on disconnect
            if "Broken pipe" not in str(e) and "Connection reset" not in str(e):
                print(f"[Command Server] Client error: {e}")
        finally:
            try:
                client_socket.close()
            except:
                pass

    def stop(self):
        """Stop server"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
        print("[Command Server] Stopped")
