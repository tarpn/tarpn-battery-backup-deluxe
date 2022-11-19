class IOProtocol:
    def handle_bytes(self, data: bytes) -> None:
        raise NotImplementedError

    def next_bytes_to_write(self, max_bytes: int) -> bytes:
        raise NotImplementedError