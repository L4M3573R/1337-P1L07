"""Metering uses actual hardware owner with synthetic modem/settings boundaries."""
from unittest.mock import Mock, patch

import pytest

from openpilot.common.esim.base import Profile, WEBBING_ICCID_PREFIX
from openpilot.common.hardware.comma.hardware import HardwareComma, NetworkType


@pytest.mark.parametrize('network', [NetworkType.cell2G, NetworkType.cell3G, NetworkType.cell4G, NetworkType.cell5G])
def test_prime_cellular_remains_metered_even_when_user_setting_is_false(network):
  hardware = HardwareComma.__new__(HardwareComma)
  hardware.get_modem_state = Mock(return_value={'iccid': WEBBING_ICCID_PREFIX + '123456789'})
  with patch('openpilot.common.params.Params') as params:
    assert hardware.get_network_metered(network) is True
  params.assert_not_called()


@pytest.mark.parametrize('modem', [{}, {'iccid': ''}, {'iccid': 'other-sim'}])
@pytest.mark.parametrize('metered', [False, True])
def test_other_cellular_retains_saved_metered_setting(modem, metered):
  hardware = HardwareComma.__new__(HardwareComma)
  hardware.get_modem_state = Mock(return_value=modem)
  with patch('openpilot.common.params.Params') as params:
    params.return_value.get_bool.return_value = metered
    assert hardware.get_network_metered(NetworkType.cell4G) is metered
  params.return_value.get_bool.assert_called_once_with('GsmMetered')


def test_wifi_never_borrows_cellular_prime_override():
  hardware = HardwareComma.__new__(HardwareComma)
  hardware.get_modem_state = Mock(side_effect=AssertionError('Wi-Fi must not inspect SIM'))
  with patch('openpilot.common.hardware.comma.hardware.wpa_supplicant_cmd', return_value={}), \
       patch('openpilot.common.hardware.base.HardwareBase.get_network_metered', return_value=False):
    assert hardware.get_network_metered(NetworkType.wifi) is False
  hardware.get_modem_state.assert_not_called()


@pytest.mark.parametrize('provider,iccid,expected', [
  ('Webbing', WEBBING_ICCID_PREFIX + '1234', True),
  ('Other', WEBBING_ICCID_PREFIX + '1234', False),
  ('Webbing', 'other-sim', False),
])
def test_comma_profile_still_requires_provider_and_shared_prefix(provider, iccid, expected):
  assert Profile(iccid, '', True, provider).is_comma is expected


def _keyfiles(tmp_path, files):
  d = tmp_path / 'nm'
  d.mkdir(exist_ok=True)
  for name, body in files.items():
    (d / name).write_text(body)
  return d


def _wifi_hardware(keyfile_dir):
  hardware = HardwareComma.__new__(HardwareComma)
  hardware.NM_KEYFILE_DIRS = (str(keyfile_dir),)
  return hardware


def _sudo_read_counter():
  calls = []
  def read(path):
    calls.append(path)
    with open(path) as f:
      return f.read()
  return calls, read


def test_wifi_metered_keyfiles_are_read_once_while_nothing_changes(tmp_path):
  # hardwared asks every 10 s; four sudo processes each time lined up with pandad CAN gaps (2026-11-19).
  d = _keyfiles(tmp_path, {'home.nmconnection': '[wifi]\nssid=home\n[connection]\nmetered=1\n',
                           'other.nmconnection': '[wifi]\nssid=other\n'})
  hardware = _wifi_hardware(d)
  calls, read = _sudo_read_counter()
  with patch('openpilot.common.hardware.comma.hardware.wpa_supplicant_cmd', return_value={'ssid': 'home'}), \
       patch('openpilot.common.hardware.comma.hardware.sudo_read', side_effect=read):
    for _ in range(5):
      assert hardware.get_network_metered(NetworkType.wifi) is True
  assert 0 < len(calls) <= 2


def test_wifi_metered_rereads_when_the_ssid_or_a_keyfile_changes(tmp_path):
  d = _keyfiles(tmp_path, {'home.nmconnection': '[wifi]\nssid=home\n[connection]\nmetered=1\n',
                           'phone.nmconnection': '[wifi]\nssid=phone\n[connection]\nmetered=2\n'})
  hardware = _wifi_hardware(d)
  calls, read = _sudo_read_counter()
  with patch('openpilot.common.hardware.comma.hardware.sudo_read', side_effect=read):
    with patch('openpilot.common.hardware.comma.hardware.wpa_supplicant_cmd', return_value={'ssid': 'home'}):
      assert hardware.get_network_metered(NetworkType.wifi) is True
    with patch('openpilot.common.hardware.comma.hardware.wpa_supplicant_cmd', return_value={'ssid': 'phone'}):
      assert hardware.get_network_metered(NetworkType.wifi) is False
    (d / 'phone.nmconnection').write_text('[wifi]\nssid=phone\n[connection]\nmetered=1\n# edited\n')
    with patch('openpilot.common.hardware.comma.hardware.wpa_supplicant_cmd', return_value={'ssid': 'phone'}):
      assert hardware.get_network_metered(NetworkType.wifi) is True


def test_wifi_metered_unknown_falls_back_to_networkmanager_and_is_cached(tmp_path):
  d = _keyfiles(tmp_path, {'home.nmconnection': '[wifi]\nssid=home\n'})
  hardware = _wifi_hardware(d)
  calls, read = _sudo_read_counter()
  with patch('openpilot.common.hardware.comma.hardware.wpa_supplicant_cmd', return_value={'ssid': 'home'}), \
       patch('openpilot.common.hardware.comma.hardware.sudo_read', side_effect=read), \
       patch('openpilot.common.hardware.base.HardwareBase.get_network_metered', return_value=False):
    assert hardware.get_network_metered(NetworkType.wifi) is False
    assert hardware.get_network_metered(NetworkType.wifi) is False
  assert len(calls) == 1


def test_an_unreadable_keyfile_is_not_remembered(tmp_path):
  d = _keyfiles(tmp_path, {'home.nmconnection': '[wifi]\nssid=home\n[connection]\nmetered=1\n'})
  hardware = _wifi_hardware(d)
  reads = iter(['', '[wifi]\nssid=home\n[connection]\nmetered=1\n'])
  with patch('openpilot.common.hardware.comma.hardware.wpa_supplicant_cmd', return_value={'ssid': 'home'}), \
       patch('openpilot.common.hardware.comma.hardware.sudo_read', side_effect=lambda p: next(reads)), \
       patch('openpilot.common.hardware.base.HardwareBase.get_network_metered', return_value=False):
    assert hardware.get_network_metered(NetworkType.wifi) is False  # sudo failed once
    assert hardware.get_network_metered(NetworkType.wifi) is True   # tried again, not cached
