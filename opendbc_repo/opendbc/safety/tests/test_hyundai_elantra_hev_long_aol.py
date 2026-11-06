"""Elantra Hybrid 2024 (camera SCC, CAN refresh) alpha-long AOL.

Main on grants lateral only, SET/RES adds longitudinal, brake drops only
longitudinal, and main off revokes both. The LDA button is an independent
lateral gesture that still requires the tracked main toggle.
"""
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests import test_hyundai_classic_long_aol as classic

ORDINARY = 0x800E
WORD = 0x8C0E  # AOL marker + LDA button
WORD_NO_LDA = 0x840E
SET, CANCEL = 2, 4


class TestHyundaiElantraHevLongAol(unittest.TestCase):
  setUp = classic.TestHyundaiClassicLongAol.setUp
  tearDown = classic.TestHyundaiClassicLongAol.tearDown
  reset = classic.TestHyundaiClassicLongAol.reset
  rx = classic.TestHyundaiClassicLongAol.rx
  request = classic.TestHyundaiClassicLongAol.request

  def feed(self, *, button=0, main=False, brake=False, available=True, lda=None):
    self.safety.set_timer(self.now)
    c = self.counter
    self.counter += 1
    self.rx('E_EMS11', {'Accel_Pedal_Pos': 0, 'CR_Vcu_AccPedDep_Pos': 0})
    self.rx('WHL_SPD11', {'WHL_SPD_AliveCounter_LSB': c % 4, 'WHL_SPD_AliveCounter_MSB': (c // 4) % 4}, integrity=True)
    self.rx('TCS13', {'AliveCounterTCS': c % 8, 'ACCEnable': 0 if available else 1, 'DriverOverride': 2 if brake else 0}, integrity=True)
    self.rx('MDPS12', {'CR_Mdps_StrColTq': 0})
    self.rx('CLU11', {'CF_Clu_AliveCnt1': c % 16, 'CF_Clu_CruiseSwState': button, 'CF_Clu_CruiseSwMain': int(main)})
    if lda is not None:
      self.rx('BCM_PO_11', {'LDA_BTN': int(lda)})
    self.now += 10000

  def settle(self, frames=6, **kwargs):
    for _ in range(frames):
      self.feed(**kwargs)

  def press(self, **kwargs):
    self.feed(**kwargs)
    self.settle(2)

  def main_on(self, word=WORD):
    self.reset(word)
    self.settle()
    self.press(main=True)

  def lkas_tx(self, torque=1):
    # A nonzero torque request is only accepted with lateral permission.
    return self.safety.safety_tx_hook(self.packer.make_can_msg_safety('LKAS11', 0, {'CR_Lkas_StrToqReq': torque, 'CF_Lkas_ActToi': 1}))

  def setUp_debug(self):
    if self.release:
      self.skipTest('Classic OP-long is DEBUG-only')

  def test_main_on_grants_lateral_only(self):
    self.setUp_debug()
    for word in (WORD, WORD_NO_LDA):
      with self.subTest(word=hex(word)):
        self.main_on(word)
        self.assertEqual(self.request(3), 1)
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertTrue(self.lkas_tx())

  def test_set_engages_long_and_brake_keeps_lateral(self):
    self.setUp_debug()
    self.main_on()
    self.press(button=SET)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertEqual(self.request(3), 3)
    self.feed(brake=True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.request(3), 1, 'Brake withdraws longitudinal only')
    self.settle(brake=True)
    self.settle()
    self.assertEqual(self.request(3), 1, 'Lateral stays through and after the brake press')
    self.assertTrue(self.lkas_tx())
    self.press(button=SET)
    self.assertEqual(self.request(3), 3, 'SET re-engages longitudinal')

  def test_cancel_keeps_lateral(self):
    self.setUp_debug()
    self.main_on()
    self.press(button=SET)
    self.assertEqual(self.request(3), 3)
    self.press(button=CANCEL)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.request(3), 1)

  def test_main_off_revokes_both_and_main_on_restores_lateral(self):
    self.setUp_debug()
    self.main_on()
    self.press(button=SET)
    self.assertEqual(self.request(3), 3)
    self.press(main=True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.request(3), 0)
    self.assertFalse(self.lkas_tx())
    self.request(0)  # host stops requesting once main is off
    self.press(main=True)
    self.assertEqual(self.request(1), 1)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_set_without_main_grants_nothing(self):
    self.setUp_debug()
    self.reset(WORD)
    self.settle()
    self.press(button=SET)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.request(3), 0)

  def test_acc_fault_revokes_lateral(self):
    self.setUp_debug()
    self.main_on()
    self.assertEqual(self.request(1), 1)
    self.feed(available=False)
    self.assertEqual(self.request(3), 0)

  def test_lda_button_grants_lateral_only_while_main_on(self):
    self.setUp_debug()
    self.reset(WORD)
    self.settle(lda=False)
    self.press(lda=True)
    self.press(lda=False)
    self.assertEqual(self.request(1), 0, 'LDA cannot grant lateral while main is off')
    self.request(1)  # host already requesting, so the main press itself grants no token
    self.press(main=True, lda=False)
    self.assertEqual(self.request(1), 0)
    self.request(0)
    self.press(lda=True)
    self.feed(lda=False)
    self.assertEqual(self.request(3), 1)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_lda_ignored_without_lda_word(self):
    self.setUp_debug()
    self.reset(WORD_NO_LDA)
    self.settle(lda=False)
    self.request(1)
    self.press(main=True, lda=False)
    self.request(0)
    self.press(lda=True)
    self.assertEqual(self.request(1), 0)

  def test_exact_words_and_experience(self):
    for experience in (0, 1, 33):
      self.safety.set_alternative_experience(experience)
      self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, WORD), 0)
      self.safety.init_tests()
      self.assertFalse(self.safety.safety_config_valid())
      self.assertEqual(self.request(3), 0)
    for word in (0x8C06, 0x8C0F, 0x9C0E, 0x0C0E, 0x8C2E):
      with self.subTest(word=hex(word)):
        self.reset(word)
        self.settle()
        self.press(main=True)
        self.assertEqual(self.request(3), 0)

  def test_release_marked_words_have_no_admission(self):
    if not self.release:
      self.skipTest('Explicit release boundary')
    for word in (WORD, WORD_NO_LDA):
      self.reset(word)
      self.assertFalse(self.safety.safety_config_valid())
      self.assertEqual(self.request(3), 0)
      self.assertFalse(self.lkas_tx())

  def test_ordinary_long_profile_is_unchanged(self):
    self.setUp_debug()
    self.reset(ORDINARY, experience=0)
    self.settle()
    self.press(main=True)
    self.assertEqual(self.request(3), 0, 'Ordinary LONG has no AOL policy')
    self.press(button=SET)
    self.assertTrue(self.safety.get_controls_allowed())


if __name__ == '__main__':
  unittest.main()
