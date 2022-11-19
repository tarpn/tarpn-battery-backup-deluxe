from importlib_metadata import version, PackageNotFoundError

try:
    __version__ = version("tarpn-bbd-agent")
except PackageNotFoundError:
    # package is not installed
    pass
