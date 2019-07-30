import sys
import socket
import selectors
import traceback
import json
import io
import struct
import threading
from queue import Queue



class BaseServer:
    '''
    Class to create a server to accumulate data from the pod and feed it to the
    GUI. If the server is initialized with with a host and port it will
    automatically start the server, otherwise, start_server will need to be
    called on the instance.

    Inputs:
        host (str):     hostname or IP address
        port (int):     port for the socket, should be above 1024
        log (str):      filepath to log incoming data, NOT IMPLEMENTED
        print_data (bool): whether or not to print data that has been received
    '''
    def __init__(self, host, port, **kwargs):
        self.sel = None
        self.print_data = kwargs.get('print_data', False)
        self.verbose = kwargs.get('verbose', False)

        self.host = host
        self.port = port

    def start_server(self):
        self.sel = selectors.DefaultSelector()

        lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Avoid bind() exception: OSError: [Errno 48] Address already in use
        lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        lsock.bind((self.host, self.port))
        lsock.listen()
        print("listening on", (self.host, self.port))
        lsock.setblocking(False)
        self.sel.register(lsock, selectors.EVENT_READ, data=None)

        try:
            while True:
                events = self.sel.select(timeout=None)
                for key, mask in events:
                    if key.data is None:
                        self._accept_wrapper(key.fileobj)
                    else:
                        message = key.data
                        try:
                            message.process_events(mask)
                            if self.print_data:
                                self._print_data(message.request.get('value'))
                            ### add code to send data to local stack ###
                            ## write data to file
                        except Exception:
                            print(
                                "main: error: exception for",
                                "{addr}:\n{traceback}"\
                                .format(addr=message.addr, traceback=traceback.format_exc()),
                            )
                            message.close()
        except KeyboardInterrupt:
            print("caught keyboard interrupt, exiting")
        finally:
            self.sel.close()

    def _accept_wrapper(self, sock):
        conn, addr = sock.accept()  # Should be ready to read
        if self.verbose:
            print("accepted connection from", addr)
        conn.setblocking(False)
        message = Message(self.sel, conn, addr)
        self.sel.register(conn, selectors.EVENT_READ, data=message)

    def _write_file(self):
        pass

    def _print_data(self, data):
        print(data)


class Message:
    '''

    '''
    def __init__(self, selector, sock, addr, verbose=False):
        self.selector = selector
        self.sock = sock
        self.addr = addr
        self._recv_buffer = b""
        self._send_buffer = b""
        self._jsonheader_len = None
        self.jsonheader = None
        self.request = None
        self.response_created = False

        self.verbose = verbose

    def _set_selector_events_mask(self, mode):
        """Set selector to listen for events: mode is 'r', 'w', or 'rw'."""
        if mode == "r":
            events = selectors.EVENT_READ
        elif mode == "w":
            events = selectors.EVENT_WRITE
        elif mode == "rw":
            events = selectors.EVENT_READ | selectors.EVENT_WRITE
        else:
            raise ValueError("Invalid events mask mode {}.".format(repr(mode)))
        self.selector.modify(self.sock, events, data=self)

    def _read(self):
        try:
            # Should be ready to read
            data = self.sock.recv(4096)
        except BlockingIOError:
            # Resource temporarily unavailable (errno EWOULDBLOCK)
            pass
        else:
            if data:
                self._recv_buffer += data
            else:
                raise RuntimeError("Peer closed.")

    def _write(self):
        if self._send_buffer:
            if self.verbose:
                print("sending", repr(self._send_buffer), "to", self.addr)
            try:
                # Should be ready to write
                sent = self.sock.send(self._send_buffer)
            except BlockingIOError:
                # Resource temporarily unavailable (errno EWOULDBLOCK)
                pass
            else:
                self._send_buffer = self._send_buffer[sent:]
                # Close when the buffer is drained. The response has been sent.
                if sent and not self._send_buffer:
                    self.close()

    def _json_encode(self, obj, encoding):
        return json.dumps(obj, ensure_ascii=False).encode(encoding)

    def _json_decode(self, json_bytes, encoding):
        tiow = io.TextIOWrapper(
            io.BytesIO(json_bytes), encoding=encoding, newline=""
        )
        obj = json.load(tiow)
        tiow.close()
        return obj

    def _create_message(
        self, *, content_bytes, content_type, content_encoding
    ):
        jsonheader = {
            "byteorder": sys.byteorder,
            "content-type": content_type,
            "content-encoding": content_encoding,
            "content-length": len(content_bytes),
        }
        jsonheader_bytes = self._json_encode(jsonheader, "utf-8")
        message_hdr = struct.pack(">H", len(jsonheader_bytes))
        message = message_hdr + jsonheader_bytes + content_bytes
        return message

    def _create_response_json_content(self):
        action = self.request.get("action")
        if action == 'send_data':
            # data = self.request.get('value')
            if self.verbose:
                print('Data:\t',type(self.request.get('value')))
            answer = 'Data Received'
            content = {'result': answer}
        else:
            content = {"result": 'Error: invalid action "{}".'.format(action)}
        content_encoding = "utf-8"
        response = {
            "content_bytes": self._json_encode(content, content_encoding),
            "content_type": "text/json",
            "content_encoding": content_encoding,
        }
        return response

    def _create_response_binary_content(self):
        response = {
            "content_bytes": b"First 10 bytes of request: "
            + self.request[:10],
            "content_type": "binary/custom-server-binary-type",
            "content_encoding": "binary",
        }
        return response

    def process_events(self, mask):
        if mask & selectors.EVENT_READ:
            self.read()
        if mask & selectors.EVENT_WRITE:
            self.write()

    def read(self):
        self._read()

        if self._jsonheader_len is None:
            self.process_protoheader()

        if self._jsonheader_len is not None:
            if self.jsonheader is None:
                self.process_jsonheader()

        if self.jsonheader:
            if self.request is None:
                self.process_request()

    def write(self):
        if self.request:
            if not self.response_created:
                self.create_response()

        self._write()

    def close(self):
        if self.verbose:
            print("closing connection to", self.addr)
        try:
            self.selector.unregister(self.sock)
        except Exception as e:
            print(
                "error: selector.unregister() exception for",
                "{}: {}".format(self.addr, repr(e)),
            )

        try:
            self.sock.close()
        except OSError as e:
            print(
                "error: socket.close() exception for",
                "{}: {}".format(self.addr, repr(e)),
            )
        finally:
            # Delete reference to socket object for garbage collection
            self.sock = None

    def process_protoheader(self):
        hdrlen = 2
        if len(self._recv_buffer) >= hdrlen:
            self._jsonheader_len = struct.unpack(
                ">H", self._recv_buffer[:hdrlen]
            )[0]
            self._recv_buffer = self._recv_buffer[hdrlen:]

    def process_jsonheader(self):
        hdrlen = self._jsonheader_len
        if len(self._recv_buffer) >= hdrlen:
            self.jsonheader = self._json_decode(
                self._recv_buffer[:hdrlen], "utf-8"
            )
            self._recv_buffer = self._recv_buffer[hdrlen:]
            for reqhdr in (
                "byteorder",
                "content-length",
                "content-type",
                "content-encoding",
            ):
                if reqhdr not in self.jsonheader:
                    raise ValueError('Missing required header "{}".'.format(reqhdr))

    def process_request(self):
        content_len = self.jsonheader["content-length"]
        if not len(self._recv_buffer) >= content_len:
            return
        data = self._recv_buffer[:content_len]
        self._recv_buffer = self._recv_buffer[content_len:]
        if self.jsonheader["content-type"] == "text/json":
            encoding = self.jsonheader["content-encoding"]
            self.request = self._json_decode(data, encoding)
            # print("received request", repr(self.request), "from", self.addr)
            if self.verbose:
                print("received request, action:", self.request.get('action'),
                    "from", self.addr)
        else:
            # Binary or unknown content-type
            self.request = data
            if self.verbose:
                print(
                'received {} request from'.format(self.jsonheader["content-type"]),
                self.addr,
                )
        # Set selector to listen for write events, we're done reading.
        self._set_selector_events_mask("w")

    def create_response(self):
        if self.jsonheader["content-type"] == "text/json":
            response = self._create_response_json_content()
        else:
            # Binary or unknown content-type
            response = self._create_response_binary_content()
        message = self._create_message(**response)
        self.response_created = True
        self._send_buffer += message

class ThreadedServer(threading.Thread, BaseServer):
    def __init__(self, q, **kwargs):
        threading.Thread.__init__(self)
        BaseServer.__init__(self, **kwargs)

        self.q = q
    
    def run(self):
        # run the server and add received data to queue
        self.sel = selectors.DefaultSelector()

        lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Avoid bind() exception: OSError: [Errno 48] Address already in use
        lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        lsock.bind((self.host, self.port))
        lsock.listen()
        print("listening on", (self.host, self.port))
        lsock.setblocking(False)
        self.sel.register(lsock, selectors.EVENT_READ, data=None)

        try:
            while True:
                events = self.sel.select(timeout=None)
                for key, mask in events:
                    if key.data is None:
                        self._accept_wrapper(key.fileobj)
                    else:
                        message = key.data
                        try:
                            message.process_events(mask)
                            if self.print_data:
                                self._print_data(message.request.get('value'))
                            self.q.put(message.request.get('value'))
                            ## write data to file (not implemented)
                        except Exception:
                            print(
                                "main: error: exception for",
                                "{addr}:\n{traceback}"\
                                .format(addr=message.addr, traceback=traceback.format_exc()),
                            )
                            message.close()
        except KeyboardInterrupt:
            print("caught keyboard interrupt, exiting")
        finally:
            self.sel.close()


if __name__ == "__main__":
    import argparse
    import time

    parser = argparse.ArgumentParser(description='Pod Data Simulator')
    parser.add_argument(
        '-p', help='print received data', action='store_true', default=False
    )
    parser.add_argument('--server', help='<host>:<port>')
    parser.add_argument(
        '-t', help='use Threaded server', action='store_true', default=False
    )
    args = parser.parse_args()

    if args.server:
        host, port = args.server.split(':')
        port = int(port)
    else:
        host, port = ('localhost', 5000)

    
    if args.t:
        q = Queue()
        serv = ThreadedServer(q, host=host, port=port, print_data=args.p)
        serv.setDaemon(True)
        serv.start()
        for i in range(200):
            if not q.empty():
                print(q.get())
            time.sleep(2)
    else:
        serv = BaseServer(host=host, port=port, print_data=args.p)
        serv.start_server()
