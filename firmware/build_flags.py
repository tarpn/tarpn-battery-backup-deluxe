import datetime
import re

# This script produces variables that are included in the compilation phase of 
# the firmware build which allows us to include build-time information in the 
# firmware binaries. 
# 
# See https://docs.platformio.org/en/latest/projectconf/section_env_build.html#built-in-variables

TAG_REGEX = re.compile(
    r"^firmware-(?:[\w-]+-)?(?P<version>[vV]?\d+(?:\.\d+){0,2}[^\+]*)(?:\+.*)?$"
)

if __name__ == "__main__":
    try:
        import setuptools_scm
        ver = setuptools_scm.get_version(root="..", tag_regex=TAG_REGEX, git_describe_command="git describe --tags --match firmware-*")
    except ImportError:
        ver = "??.??"
    now = datetime.datetime.utcnow().replace(microsecond=0).isoformat()
    print(f"-DBUILD_VERSION='\"{ver}\"' -DBUILD_DATE='\"{now}Z\"'")
