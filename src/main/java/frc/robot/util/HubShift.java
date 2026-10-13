package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Tracks when our alliance's hub is active during teleop.
 *
 * <p>Teleop (140s) is split into: TRANSITION (both hubs active), SHIFTS 1-4 (hubs alternate), and
 * END GAME (both hubs active). The FMS sends 'R' or 'B' as game data ~3s after auto, naming the
 * alliance whose hub goes inactive first (the one that scored more fuel in auto). That alliance is
 * inactive in shifts 1 and 3, active in shifts 2 and 4.
 *
 * <p>Times below are teleop match time remaining, from docs.wpilib.org 2026 Game Data Details.
 * Verify against the game manual.
 */
public final class HubShift {
  private static final double TRANSITION_END = 130.0;
  private static final double SHIFT_LENGTH = 25.0;
  private static final double ENDGAME_START = 30.0;

  /** Match times where the active hub can change, in the order they happen. */
  private static final double[] BOUNDARIES = {130.0, 105.0, 80.0, 55.0, 30.0};

  private HubShift() {}

  /**
   * True when we know enough to track shifts: in teleop, with a real countdown (FMS or DS practice
   * mode, otherwise getMatchTime() is -1) and game data received.
   */
  public static boolean isKnown() {
    return DriverStation.isTeleop()
        && DriverStation.getMatchTime() >= 0
        && getInactiveFirstAlliance() != null;
  }

  /** True if our hub is active right now. Assumes active when shifts aren't known. */
  public static boolean isOurHubActive() {
    if (!isKnown()) return true;
    return isActiveAt(DriverStation.getMatchTime());
  }

  /** True during the last 30 seconds of teleop. */
  public static boolean isEndgame() {
    double time = DriverStation.getMatchTime();
    return DriverStation.isTeleop() && time >= 0 && time <= ENDGAME_START;
  }

  /** Seconds left in endgame, or 0 when not in endgame. */
  public static double getEndgameSecondsLeft() {
    return isEndgame() ? DriverStation.getMatchTime() : 0.0;
  }

  /** Seconds until our hub becomes active, or 0 if it's active now (or unknown). */
  public static double getSecondsUntilActive() {
    if (!isKnown()) return 0.0;
    double time = DriverStation.getMatchTime();
    if (isActiveAt(time)) return 0.0;
    for (double boundary : BOUNDARIES) {
      if (boundary < time && isActiveAt(boundary - 0.01)) return time - boundary;
    }
    return 0.0;
  }

  /** Seconds until our hub goes inactive (or the match ends), or 0 if it's inactive now. */
  public static double getSecondsLeftActive() {
    if (!isKnown()) return 0.0;
    double time = DriverStation.getMatchTime();
    if (!isActiveAt(time)) return 0.0;
    for (double boundary : BOUNDARIES) {
      if (boundary < time && !isActiveAt(boundary - 0.01)) return time - boundary;
    }
    return time; // active until the end of the match
  }

  /** Whether our hub is active at the given teleop match time remaining. */
  private static boolean isActiveAt(double time) {
    if (time > TRANSITION_END || time <= ENDGAME_START) return true;

    int shift = (int) ((TRANSITION_END - time) / SHIFT_LENGTH) + 1; // 1 to 4
    boolean shiftIsOdd = shift % 2 == 1;
    boolean weGoInactiveFirst = getInactiveFirstAlliance() == getOurAlliance();
    return weGoInactiveFirst ? !shiftIsOdd : shiftIsOdd;
  }

  /** The alliance whose hub goes inactive first, from FMS game data. Null if not sent yet. */
  private static Alliance getInactiveFirstAlliance() {
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) return null;
    return switch (gameData.charAt(0)) {
      case 'R' -> Alliance.Red;
      case 'B' -> Alliance.Blue;
      default -> null;
    };
  }

  private static Alliance getOurAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }
}
