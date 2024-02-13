import os
import re

SEEREP_CFG_PATH = "seerep.cfg"
DATA_FOLDER_CONFIG_KEY = "data-folder"
LOG_FOLDER_CONFIG_KEY = "log-path"
PORT_CONFIG_KEY = "port"

# read relevant values from seerep.cfg
def setup_test_struct():
    # this mimics the behaviour of persistent variables inside functions, as everything in python is a object
    if not hasattr(setup_test_struct, "kv_dict"):
        # todo: use a more robust method to find/read the config file
        # searches seerep.cfg relative to the location of this file
        with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), SEEREP_CFG_PATH), "r") as f:
            lines = f.read().split("\n")

        kv_dict = {}
        for kv_wcomments in lines:
            kv_no_spaces = kv_wcomments.replace(" ", "")
            # get rid of comments
            kv_pairs = kv_no_spaces.split("#")[0]
            # skip if there is no = in the relevant string
            if "=" not in kv_pairs:
                continue
            kv_tuple = kv_pairs.split("=", maxsplit=1)

            try:
                kv_dict[kv_tuple[0].strip()] = int(kv_tuple[1])
            except ValueError:
                kv_dict[kv_tuple[0].strip()] = kv_tuple[1].strip()

        if PORT_CONFIG_KEY not in kv_dict or not isinstance(kv_dict[PORT_CONFIG_KEY], int):
            raise ValueError(
                f"wrong formatting of {SEEREP_CFG_PATH}. Port option 'port' is missing. (default: {PORT_CONFIG_KEY} = 9095)"
            )

        if DATA_FOLDER_CONFIG_KEY not in kv_dict:
            raise ValueError(
                f"wrong formatting of {SEEREP_CFG_PATH}. Data folder option 'data-folder' is missing or is not a valid folder on the filesystem. (default: {DATA_FOLDER_CONFIG_KEY} = /seerep/seerep-data/tests/)"
            )
        elif not os.path.isdir(kv_dict[DATA_FOLDER_CONFIG_KEY]):
            os.makedirs(kv_dict[DATA_FOLDER_CONFIG_KEY], exist_ok=True)

        if LOG_FOLDER_CONFIG_KEY not in kv_dict:
            kv_dict[LOG_FOLDER_CONFIG_KEY] = None
        elif not os.path.isdir(kv_dict[LOG_FOLDER_CONFIG_KEY]):
            os.makedirs(kv_dict[DATA_FOLDER_CONFIG_KEY], exist_ok=True)
            # raise ValueError(f"wrong formatting of {SEEREP_CFG_PATH}. Log folder option 'log-folder' is set but not a valid folder on the filesystem. (default: {LOG_FOLDER_CONFIG_KEY} = /seerep/seerep-data/tests/log/)")

        setup_test_struct.kv_dict = kv_dict

    return (
        setup_test_struct.kv_dict[PORT_CONFIG_KEY],
        setup_test_struct.kv_dict[DATA_FOLDER_CONFIG_KEY],
        setup_test_struct.kv_dict[LOG_FOLDER_CONFIG_KEY],
    )


def cleanup_test_remnants():
    data_folder = setup_test_struct()[1]
    log_folder = setup_test_struct()[2]

    # pattern source https://stackoverflow.com/questions/11384589/what-is-the-correct-regex-for-matching-values-generated-by-uuid-uuid4-hex
    project_files = [
        f
        for f in os.listdir(data_folder)
        if re.search(r"[0-9a-f]{8}\-[0-9a-f]{4}\-4[0-9a-f]{3}\-[89ab][0-9a-f]{3}\-[0-9a-f]{12}\.h5", f)
        and os.path.isfile(f"{data_folder}{f}")
    ]

    if len(project_files) > 0 and input(f"Detected leftover projects in {data_folder}. Delete them? (y/N): ") == "y":
        for f in project_files:
            os.remove(f"{data_folder}{f}")

    # remove older log entries
    if setup_test_struct != None:
        log_files = [
            f
            for f in os.listdir(log_folder)
            if re.search("seerep_[1-9]\\d*\\.log", f) and os.path.isfile(f"{log_folder}{f}")
        ]
        if len(log_files) > 0 and input(f"Detected older log-files in {log_folder}. Delete them? (y/N): ") == "y":
            for f in log_files:
                os.remove(f"{log_folder}{f}")


if __name__ == "__main__":
    cleanup_test_remnants()
