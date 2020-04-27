import os

TEMP_PATH_SUFFIX = "_tempAPI"


def safe_read_file(file_path):

    while True:

        try:

            file = open(file_path, "r")
            break

        except EnvironmentError as e:

            continue

    result = file.read()
    file.close()
    return result


def safe_override_or_create_file_through_temp(text, file_path):

    temp_file_path = file_path + TEMP_PATH_SUFFIX

    temp_file = open(temp_file_path, "w")
    temp_file.write(text)
    temp_file.close()

    wait_and_move_temp_file_back(file_path)


def wait_and_move_temp_file_back(file_path):

    while True:

        try:

            os.replace(file_path + TEMP_PATH_SUFFIX, file_path)
            break

        except EnvironmentError as e:

            continue
