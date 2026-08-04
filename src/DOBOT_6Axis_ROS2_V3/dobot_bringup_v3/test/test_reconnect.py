from dobot_bringup_v3.dobot_bringup import _RetryingDobotProxy


class _FakeApi:
    def __init__(self, reply=None, error=None):
        self.reply = reply
        self.error = error
        self.closed = False

    def RobotMode(self):
        if self.error is not None:
            raise self.error
        return self.reply

    def close(self):
        self.closed = True


def test_proxy_returns_offline_result_and_requests_reconnect():
    api = _FakeApi(error=TimeoutError('controller offline'))
    failures = []
    proxy = _RetryingDobotProxy(
        api,
        lambda connection, command, error: failures.append(
            (connection, command, error)
        ),
    )

    assert proxy.RobotMode() == '-1,{},RobotMode();'
    assert len(failures) == 1
    assert failures[0][0] is proxy
    assert failures[0][1] == 'RobotMode'
    assert isinstance(failures[0][2], TimeoutError)


def test_proxy_treats_empty_reply_as_disconnect():
    api = _FakeApi(reply='')
    failures = []
    proxy = _RetryingDobotProxy(api, lambda *args: failures.append(args))

    assert proxy.RobotMode() == '-1,{},RobotMode();'
    assert len(failures) == 1


def test_proxy_passes_valid_reply_unchanged():
    api = _FakeApi(reply='0,{5},RobotMode();')
    failures = []
    proxy = _RetryingDobotProxy(api, lambda *args: failures.append(args))

    assert proxy.RobotMode() == '0,{5},RobotMode();'
    assert failures == []
