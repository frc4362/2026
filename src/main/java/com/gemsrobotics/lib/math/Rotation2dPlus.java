
package com.gemsrobotics.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import static java.lang.Math.abs;

public final class Rotation2dPlus extends Rotation2d {
	public Rotation2dPlus() {
		super();
	}

	public Rotation2dPlus(final double radians) {
		super(radians);
	}

	public Rotation2dPlus(final Rotation2d rotation) {
		super(rotation.getRadians());
	}

	public Rotation2dPlus(final double x, final double y) {
		super(x, y);
	}

	public static Rotation2dPlus fromRadians(final double radians) {
		return new Rotation2dPlus(radians);
	}

	public static Rotation2dPlus fromDegrees(final double degrees) {
		return fromRadians(Units.degreesToRadians(degrees));
	}

	public static Rotation2dPlus fromRotations(final double rotations) {
		return fromRadians(rotations * 2 * Math.PI);
	}

	private static final List<Rotation2dPlus> sixthsEastWest = List.of(
			Rotation2dPlus.fromDegrees(30),
			Rotation2dPlus.fromDegrees(90),
			Rotation2dPlus.fromDegrees(150),
			Rotation2dPlus.fromDegrees(210),
			Rotation2dPlus.fromDegrees(270),
			Rotation2dPlus.fromDegrees(330)
			);

	/**
	 * @return The pole nearest to this rotation.
	 */
	public Rotation2dPlus getNearestPole() {
		final double poleSin;
		final double poleCos;

		if (abs(getCos()) > abs(getSin())) {
			poleCos = Math.signum(getCos());
			poleSin = 0.0;
		} else {
			poleCos = 0.0;
			poleSin = Math.signum(getSin());
		}

		return new Rotation2dPlus(poleCos, poleSin);
	}

	public Rotation2dPlus getNearestHex() {return this.nearest(sixthsEastWest);}

	public Rotation2dPlus nearest(List<Rotation2dPlus> rotations) {
		return Collections.min(rotations, Comparator.comparing((a) -> (abs(minus(a).getDegrees()))));
	}

	private static final List<Rotation2dPlus> pickupDirection = List.of(
			Rotation2dPlus.fromDegrees(54.0),
			Rotation2dPlus.fromDegrees(-54.0),
			Rotation2dPlus.fromDegrees(234.0),
			Rotation2dPlus.fromDegrees(-234.0)
	);

	public Rotation2dPlus getNearestPickupAngle() {
		return nearest(pickupDirection);
	}
//		final double x;
//		final double y;
//		if (abs(getSin()) <= 0.5) {
//			if(getCos() > 0) { // east
//				x = 1;
//				y = 0;
//			} else { // west
//				x = -1;
//				y = 0;
//			}
//		} else if (getSin() < 0) {
//			if(getCos() > 0) { // southeast
//				x = 0.5;
//				y = -sqrt(3)/2.0;
//			} else { // southwest
//				x = -0.5;
//				y = -sqrt(3)/2.0;
//			}
//		} else {
//			if(getCos() > 0) { // northeast
//				x = 0.5;
//				y = sqrt(3)/2.0;
//			}
//			else { // northwest
//				x = -0.5;
//				y = sqrt(3)/2.0;
//			}
//		}
//		return new Rotation2dPlus(x, y);
}
