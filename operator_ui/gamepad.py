"""Lecture de la manette (pygame) dans un thread dédié. Les widgets reçoivent des PadState par
signal Qt ; aucun widget n'est touché depuis ce thread."""
from __future__ import annotations

import os
import time

from PySide6.QtCore import QThread, Signal

from .model import PadState

# Doit être défini AVANT l'import de pygame : pas de fenêtre, événements joystick sans focus.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")


class GamepadThread(QThread):
    state = Signal(object)      # PadState, ~50 Hz

    def __init__(self, rate_hz: float = 50.0, parent=None) -> None:
        super().__init__(parent)
        self._period = 1.0 / rate_hz
        self._running = True

    def stop(self) -> None:
        self._running = False
        self.wait(2000)

    def run(self) -> None:
        try:
            import pygame
        except ImportError:
            self.state.emit(PadState(error="pygame non installé (pip install pygame)"))
            return
        pygame.display.init()
        pygame.joystick.init()
        joy = None
        while self._running:
            try:
                for ev in pygame.event.get():
                    if ev.type == pygame.JOYDEVICEADDED and joy is None:
                        joy = pygame.joystick.Joystick(ev.device_index)
                    elif ev.type == pygame.JOYDEVICEREMOVED and joy is not None \
                            and joy.get_instance_id() == ev.instance_id:
                        joy = None
                if joy is None and pygame.joystick.get_count() > 0:
                    joy = pygame.joystick.Joystick(0)
                if joy is None:
                    self.state.emit(PadState(connected=False, stamp=time.monotonic()))
                else:
                    self.state.emit(PadState(
                        connected=True, name=joy.get_name(),
                        axes=tuple(joy.get_axis(i) for i in range(joy.get_numaxes())),
                        buttons=tuple(bool(joy.get_button(i)) for i in range(joy.get_numbuttons())),
                        stamp=time.monotonic()))
            except pygame.error:
                joy = None          # périphérique disparu pendant la lecture
                self.state.emit(PadState(connected=False, stamp=time.monotonic()))
            time.sleep(self._period)
        pygame.quit()


def check_cli(seconds: float = 60.0) -> int:
    """Affiche en continu axes et boutons pour identifier le mapping (sans Qt ni ROS)."""
    try:
        import pygame
    except ImportError:
        print("pygame non installé : pip install pygame")
        return 1
    pygame.display.init()
    pygame.joystick.init()
    print("Branchez la manette ; bougez les sticks / pressez les boutons (Ctrl+C pour quitter).")
    joy, last, end = None, None, time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            pygame.event.pump()
            if joy is None and pygame.joystick.get_count() > 0:
                joy = pygame.joystick.Joystick(0)
                print(f"Manette : {joy.get_name()} — {joy.get_numaxes()} axes, "
                      f"{joy.get_numbuttons()} boutons, {joy.get_numhats()} croix")
            if joy is not None:
                axes = [round(joy.get_axis(i), 2) for i in range(joy.get_numaxes())]
                btns = [i for i in range(joy.get_numbuttons()) if joy.get_button(i)]
                cur = (axes, btns)
                if cur != last:
                    print(f"axes={axes}  boutons pressés={btns}")
                    last = cur
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
    return 0
