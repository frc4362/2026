package com.gemsrobotics.shooting;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class LookupTableStrategy extends LaunchStrategy {
    public record TableEntry(double range, LaunchParameters params) {}

    private static final List<TableEntry> ENTRIES;
    private static final InterpolatingTreeMap<Double, LaunchParameters> LOOKUP_TABLE;
    static {
        ENTRIES = new ArrayList<>();
        ENTRIES.add(new TableEntry(2, new LaunchParameters(Rotation2d.fromDegrees(15.0), 25.0)));
        ENTRIES.add(new TableEntry(3, new LaunchParameters(Rotation2d.fromDegrees(25.0), 27.5)));
        ENTRIES.add(new TableEntry(4, new LaunchParameters(Rotation2d.fromDegrees(35.0), 32.5)));
        ENTRIES.add(new TableEntry(5, new LaunchParameters(Rotation2d.fromDegrees(50.0), 40.0)));
        ENTRIES.sort(Comparator.comparingDouble(TableEntry::range));

        LOOKUP_TABLE = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), LaunchParameters::interpolate);
        for (final var entry : ENTRIES) {
            LOOKUP_TABLE.put(entry.range, entry.params);
        }
    }

    @Override
    protected LaunchParameters unsafeParametersFor(final double rangeMeters) {
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
