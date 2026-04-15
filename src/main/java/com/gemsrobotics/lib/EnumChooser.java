package com.gemsrobotics.lib;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public final class EnumChooser<T extends Enum<T>> extends SendableChooser<T> {
	public EnumChooser(final Class<T> enumClass, final T defaultChoice) {
		super();

		setDefaultOption(defaultChoice.name(), defaultChoice);
		for (final T choice : enumClass.getEnumConstants()) {
			if (!choice.equals(defaultChoice)) {
				addOption(choice.name(), choice);
			}
		}
	}
}
