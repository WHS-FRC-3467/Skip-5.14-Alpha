package frc.robot.Util;

import java.util.Collections;
import java.util.List;

public class VisionLookUpTable {
    ShooterConfig shooterConfig;

    private static VisionLookUpTable instance = new VisionLookUpTable();

    public static VisionLookUpTable getInstance() {
        return instance;
    }
    public VisionLookUpTable() {
        shooterConfig = new ShooterConfig();
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 1)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 2)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 3)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 4)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 5)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 6)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 7)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 8)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 9)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 10)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 11)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 12)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 13)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0, 14)); // Distance -> Bumper

        Collections.sort(shooterConfig.getShooterConfigs());
    }

    /*
     * Obtains a shooter preset from a given target distance
     * @param DistanceFromTarget measured distance to the shooting target
     * @return new shooter preset for given distance
     */
    public ShooterPreset getShooterPreset(double DistanceFromTarget) {
        int endIndex = shooterConfig.getShooterConfigs().size()-1;

        /*
         * Check if distance falls below the shortest distance in the lookup table. If the measured distance is shorter
         * select the lookup table entry with the shortest distance
         */
        if(DistanceFromTarget <= shooterConfig.getShooterConfigs().get(0).getDistance()){
            return shooterConfig.getShooterConfigs().get(0);
        }

        /*
         * Check if distance falls above the largest distance in the lookup table. If the measured distance is larger
         * select the lookup table entry with the largest distance
         */
        if(DistanceFromTarget >= shooterConfig.getShooterConfigs().get(endIndex).getDistance()){
            return shooterConfig.getShooterConfigs().get(endIndex);
        }
        /*
         * If the measured distance falls somewhere within the lookup table perform a binary seqarch within the lookup
         * table
         */
        return binarySearchDistance(shooterConfig.getShooterConfigs(),0, endIndex, DistanceFromTarget);
    }

    /*
     * Perform fast binary search to find a matching shooter preset. if no matching preset is found it interpolates a
     * new shooter preset based on the two surrounding table entries.
     * 
     * @param ShooterConfigs: the table containing the shooter presets
     * @param StartIndex: Starting point to search
     * @param EndIndex: Ending point to search
     * @param Distance: Distance for which we need to find a preset
     * 
     * @return (Interpolated) shooting preset
     */
    private ShooterPreset binarySearchDistance(List<ShooterPreset> ShooterConfigs, int StartIndex, int EndIndex, double Distance) {
        int mid = StartIndex + (EndIndex - StartIndex) / 2;
        double midIndexDistance = ShooterConfigs.get(mid).getDistance();

        // If the element is present at the middle
        // return itself
        if (Distance == midIndexDistance) {
            return ShooterConfigs.get(mid);
        }
        // If only two elements are left
        // return the interpolated config
        if (EndIndex - StartIndex == 1) {
            double percentIn = (Distance - shooterConfig.getShooterConfigs().get(StartIndex).getDistance()) / 
                (
                    shooterConfig.getShooterConfigs().get(EndIndex).getDistance() - 
                        shooterConfig.getShooterConfigs().get(StartIndex).getDistance()
                );
            return interpolateShooterPreset(shooterConfig.getShooterConfigs().get(StartIndex), shooterConfig.getShooterConfigs().get(EndIndex), percentIn);
        }
        // If element is smaller than mid, then
        // it can only be present in left subarray
        if (Distance < midIndexDistance) {
            return binarySearchDistance(ShooterConfigs, StartIndex, mid, Distance);
        }
        // Else the element can only be present in right subarray
        return binarySearchDistance(ShooterConfigs, mid, EndIndex, Distance);
    }

    /*
     * Obtain a new shooter preset by interpolating between two existing shooter presets
     * 
     * @param StartPreset: Starting preset for interpolation
     * @param EndPreset: Ending preset for interpolation
     * @param PercentIn: Amount of percentage between the two values the new preset needs to be
     * 
     * @return new interpolated shooter preset
     */
    private ShooterPreset interpolateShooterPreset(ShooterPreset StartPreset, ShooterPreset EndPreset, double PercentIn) {
        double armAngle = StartPreset.getArmAngle() + (EndPreset.getArmAngle() - StartPreset.getArmAngle()) * PercentIn;
        double leftShooter = StartPreset.getLeftShooter() + (EndPreset.getLeftShooter() - StartPreset.getLeftShooter()) * PercentIn;
        double rightShooter = StartPreset.getRightShooter() + (EndPreset.getRightShooter() - StartPreset.getRightShooter()) * PercentIn;
        double distance = StartPreset.getDistance() + (EndPreset.getDistance() - StartPreset.getDistance()) * PercentIn;

        return new ShooterPreset(armAngle, leftShooter, rightShooter, distance);
    }

    /*
     * MAKE SURE YOU SORT THE LIST BEFORE CALLING THIS FUNCTION
     * @param pShooterConfig a sorted shooter config
     */
    public void setShooterConfig(ShooterConfig pShooterConfig) {
        this.shooterConfig = pShooterConfig;
    }
}
