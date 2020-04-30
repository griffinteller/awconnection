import win32pipe, win32file

MESSAGE_SEPARATOR = b";"


def get_most_recent_message(pipe_handle):

    """Retreives the most recent valid message from a named pipe. If there are none, it returns an empty string

    Parameters
    ----------
    pipe_handle : PyHandle
        The handle to the pipe

    Returns
    -------
    str
    """

    pipe_message_text = win32file.ReadFile(pipe_handle, 64 * 1024)[1]
    pipe_messages = pipe_message_text.split(MESSAGE_SEPARATOR)

    if len(pipe_messages) == 0:

        return b""

    if len(pipe_messages[-1]) == 0 or pipe_messages[-1][-1] != MESSAGE_SEPARATOR:

        return pipe_messages[-2]  # remove separator

    else:

        return pipe_messages[-1]
