import omni.replicator.core as rep


class DataBackend(rep.backends.BaseBackend):
    def __init__(self, output_dir: str):
        self.output_dir = output_dir

    def write_blob(self, path, data):
        return super().write_blob(path, data)

    def read_blob(self, path):
        return super().read_blob(path)


