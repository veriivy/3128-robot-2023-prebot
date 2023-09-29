package frc.team3128;

public class PositionConstants {
    public static enum Position {
        HIGH_CONE(10, 45, true),
        HIGH_CUBE(10, 45, false),
        MID_CONE(10, 45, true),
        MID_CUBE(10, 45, false),
        LOW(10, 45, true),

        SHELF_CONE(10, 45, true),
        SHELF_CUBE(10, 45, false),
        CHUTE_CONE(10, 45, true),
        CHUTE_CUBE(10, 45, false),

        GROUND_CONE(10, 45, true),
        GROUND_CUBE(10, 45, false),

        NEUTRAL(5, 45, false);

        public double elvDist;
        public double wristAngle;
        public boolean cone;

        private Position(double elvDist, double wristAngle, boolean cone) {
            this.elvDist = elvDist;
            this.wristAngle = wristAngle;
            this.cone = cone;
        }
    }
}
