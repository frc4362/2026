package com.gemsrobotics.lib;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Function;

/**
 * Lifts a set of same-typed NT4 publishers to a stateless key-value map. Taking these two type parameters is the easiest way to simulate a typeclass
 * @param <PublisherType> The type of the publisher which we are storing ie. DoublePublisher, BooleanPublisher, StructPublisher<T>
 * @param <LoggableType> The type of the value which we are logging to this publisher. ie. Double, Boolean, or T
 */
public final class StatelessNetworkTable<PublisherType extends Publisher, LoggableType> {
    private final Map<String, PublisherType> m_publishers;
    private final Function<String, PublisherType> m_publisherMaker;
    private final BiConsumer<PublisherType, LoggableType> m_logFunction;

    private StatelessNetworkTable(final Function<String, PublisherType> publisherMaker, final BiConsumer<PublisherType, LoggableType> log) {
        m_publishers = new HashMap<>();
        m_publisherMaker = publisherMaker;
        m_logFunction = log;
    }

    private PublisherType getOrMakePublisher(final String key) {
        return m_publishers.computeIfAbsent(key, m_publisherMaker);
    }

    public void logPair(final Pair<String, LoggableType> keyValue) {
        logKeyValue(keyValue.getFirst(), keyValue.getSecond());
    }

    public void logKeyValue(final String key, final LoggableType value) {
        m_logFunction.accept(getOrMakePublisher(key), value);
    }

    public static StatelessNetworkTable<StringPublisher, String> stringPublishers(final NetworkTable baseTable) {
        return new StatelessNetworkTable<>(
                topicName -> baseTable.getStringTopic(topicName).publish(),
                StringPublisher::set);
    }

    public static StatelessNetworkTable<BooleanPublisher, Boolean> booleanPublishers(final NetworkTable baseTable) {
        return new StatelessNetworkTable<>(
                topicName -> baseTable.getBooleanTopic(topicName).publish(),
                BooleanPublisher::set);
    }

    public static StatelessNetworkTable<DoublePublisher, Double> doublePublishers(final NetworkTable baseTable) {
        return new StatelessNetworkTable<>(
                topicName -> baseTable.getDoubleTopic(topicName).publish(),
                DoublePublisher::set);
    }

    public static StatelessNetworkTable<StructPublisher<Pose2d>, Pose2d> posePublishers(final NetworkTable baseTable) {
        return new StatelessNetworkTable<>(
                topicName -> baseTable.getStructTopic(topicName, Pose2d.struct).publish(),
                StructPublisher::set);
    }

    public static StatelessNetworkTable<StructArrayPublisher<Translation2d>, Translation2d[]> translationsPublishers(final NetworkTable baseTable) {
        return new StatelessNetworkTable<>(
                topicName -> baseTable.getStructArrayTopic(topicName, Translation2d.struct).publish(),
                StructArrayPublisher::set);
    }
}
