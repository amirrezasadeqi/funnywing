import json


class Tcp_message(object):
    def __init__(self, data=None, num_length_prefix_bytes=4):
        # The python object data, the message created for that.
        self._py_obj_data = data
        self._prefix_length = num_length_prefix_bytes
        if data is not None:
            self._encoded_data = self._encode()
        return

    def send_over(self, conn):
        # conn is a tcp socket or connection.
        conn.sendall(self._encoded_data)
        return

    def read_from(self, conn):
        length_prefix = conn.recv(self._prefix_length)
        if not length_prefix:
            return
        message_length = int(length_prefix.decode().strip())
        self._encoded_data = length_prefix + conn.recv(message_length)
        self._py_obj_data = self._decode()
        return

    def decode(self):
        return self._py_obj_data

    def _encode(self):
        message = json.dumps(self._py_obj_data).encode()
        length_prefix = f"{len(message):0{self._prefix_length}}".encode()
        return length_prefix + message

    def _decode(self):
        length_prefix = self._encoded_data[:self._prefix_length]
        message_length = int(length_prefix.decode().strip())
        message = self._encoded_data[self._prefix_length:self._prefix_length + message_length].decode()
        return json.loads(message)
