import adapters
from config import config

sma = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)
smc = sma.get_connector()
tn = smc.get_telnet()


def test_switch_to_relative():
    r = sma.try_get_response()
    assert r == ""

    r = sma.switch_to_relative()
    assert r == sma.RESPONSE_OK

    r = sma.try_get_response()
    assert r == ""


def test_halt():
    r = sma.try_get_response()
    assert r == ""

    r = sma.halt()
    assert r == "ok Emergency Stop Requested - reset or M999 required to exit HALT state\r\n"

    r = sma.try_get_response()
    assert r == ""

    r = sma.switch_to_relative()
    assert r == sma.RESPONSE_ALARM_LOCK

    r = sma.reset()
    assert r == "WARNING: After HALT you should HOME as position is currently unknown\nok\n"

    r = sma.try_get_response()
    assert r == ""

    r = sma.switch_to_relative()
    assert r == sma.RESPONSE_OK

    r = sma.try_get_response()
    assert r == ""


if __name__ == "__main__":
    test_switch_to_relative()
    test_halt()
    print("Done.")
