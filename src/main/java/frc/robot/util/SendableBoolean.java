package frc.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SendableBoolean {
    private BooleanPublisher m_publisher;
    private BooleanSubscriber m_subscriber;
    private BooleanTopic m_topic;
    private boolean m_defaultValue;

    public SendableBoolean(String topicName, boolean defaultValue, Command onPressed) {
        m_topic = NetworkTableInstance.getDefault().getBooleanTopic(topicName);
        m_publisher = m_topic.publish();
        m_publisher.setDefault(defaultValue);

        m_subscriber = m_topic.subscribe(defaultValue);
        m_defaultValue = defaultValue;

        if (onPressed != null) {
            new Trigger(this::getBoolean).onTrue(onPressed);
        }
    }

    public SendableBoolean(String topicName, boolean defaultValue) {
        this(topicName, defaultValue, null);
    }

    public boolean getBoolean() {
        return m_subscriber.get(m_defaultValue);
    }
}