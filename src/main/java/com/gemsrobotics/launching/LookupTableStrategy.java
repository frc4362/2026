package com.gemsrobotics.launching;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class LookupTableStrategy extends LaunchStrategy {
    public record TableEntry(double range, LauncherParameters params) {}

    protected static final List<TableEntry> ENTRIES;
    protected static final InterpolatingTreeMap<Double, LauncherParameters> LOOKUP_TABLE;
    static {
        ENTRIES = new ArrayList<>();
        ENTRIES.add(new TableEntry(2, new LauncherParameters(Rotation2d.fromDegrees(15.0), 25.0)));
        ENTRIES.add(new TableEntry(3, new LauncherParameters(Rotation2d.fromDegrees(25.0), 27.5)));
        ENTRIES.add(new TableEntry(4, new LauncherParameters(Rotation2d.fromDegrees(30.0), 32.5)));
        ENTRIES.add(new TableEntry(5, new LauncherParameters(Rotation2d.fromDegrees(38.0), 40.0)));
        ENTRIES.sort(Comparator.comparingDouble(TableEntry::range));

        LOOKUP_TABLE = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), LauncherParameters::interpolate);
        for (final var entry : ENTRIES) {
            LOOKUP_TABLE.put(entry.range, entry.params);
        }
    }

    @Override
    protected LauncherParameters unsafeParametersFor(final double rangeMeters) {
        return LOOKUP_TABLE.get(rangeMeters);
    }

    @Override
    public double getMinRangeMeters() {
        return ENTRIES.get(0).range;
    }

    @Override
    public double getMaxRangeMeters() {
        return ENTRIES.get(ENTRIES.size() - 1).range;
    }
}
