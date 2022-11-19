import logging
from functools import partial

logging.addLevelName(logging.DEBUG - 5, "TRACE")
TRACE = logging.DEBUG - 5
logging.TRACE = TRACE


class LoggingMixin:
    def __init__(self, logger: logging.Logger = None, extra_func=None, extra=None, stacklevel=2):
        if logger is None:
            self.logger = logging.getLogger("root")
        else:
            self.logger = logger
        if extra_func is None:
            self.extra_func = partial(str, self.__class__.__qualname__)
        else:
            self.extra_func = extra_func

        if extra:
            self.extra = extra
        else:
            self.extra = dict()

        self.stacklevel = stacklevel
        #self.error_counter = get_registry().counter("logging:error")

    def is_trace_logging(self):
        return self.logger.isEnabledFor(TRACE)

    def is_debug_logging(self) -> bool:
        return self.logger.isEnabledFor(logging.DEBUG)

    def _resolve_extra(self, kwargs):
        """
        Resolution order is: logger instance, context variables, then the kwargs from the call
        """
        extra = {}
        extra.update(self.extra)
        #extra["trace_id"] = get_trace_id()
        extra.update(kwargs.pop("extra", {}))
        kwargs["extra"] = extra
        kwargs["stacklevel"] = self.stacklevel

    def info(self, msg, *args, **kwargs):
        self._resolve_extra(kwargs)
        self.logger.info(f"{self.extra_func()} {msg}", *args, **kwargs)

    def debug(self, msg, *args, **kwargs):
        self._resolve_extra(kwargs)
        self.logger.debug(f"{self.extra_func()} {msg}", *args, **kwargs)

    def trace(self, msg, *args, **kwargs):
        self._resolve_extra(kwargs)
        self.logger.log(level=TRACE, msg=f"{self.extra_func()} {msg}", *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        self._resolve_extra(kwargs)
        self.logger.warning(f"{self.extra_func()} {msg}", *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        self._resolve_extra(kwargs)
        self.logger.error(f"{self.extra_func()} {msg}", *args, **kwargs)

    def exception(self, msg):
        self._resolve_extra({})
        self.logger.exception(f"{self.extra_func()} {msg}")
