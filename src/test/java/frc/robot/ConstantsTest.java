package frc.robot;

import static frc.robot.Constants.CanId;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;
import java.util.*;

public class ConstantsTest {

    /**
     * A CAN type and a CAN ID. This is what needs to be unique.
     */
    record CanIdAndType(CanId.Type type, int id) {}

    /**
     * Test that no two CAN ID fields for devices of the same type have the same CAN ID.
     */
    @Test
    public void testDuplicateCanIds() throws Exception {
        Map<CanIdAndType,Set<String>> canIdToFields = new HashMap<>();

        // find CAN IDs
        findCanIds(canIdToFields, Constants.class);

        // Verify that there are no duplicates
        List<String> errors = new ArrayList<>();
        canIdToFields.forEach((canIdAndType, fieldNames) -> {
            if (fieldNames.size() > 1) {
                String message = String.format("%s fields have CAN Type/ID %s/%s: %s",
                        fieldNames.size(), canIdAndType.type, canIdAndType.id, fieldNames);
                errors.add(message);
            }
        });
        String message = String.join("\n", errors);
        System.err.println(message);
        Assertions.assertEquals(0, errors.size(), message);
    }

    /**
     * Find CAN ID fields in the supplied class and add field names to the supplied map.
     * Recursively finds CAN ID fields in subclasses.
     */
    private void findCanIds(Map<CanIdAndType,Set<String>> canIdToFields, Class<?> cls)
        throws IllegalAccessException {
        // Find CAN ID fields
        for (Field field : cls.getDeclaredFields()) {
            CanId annotation = field.getAnnotation(CanId.class);
            if (annotation != null) {
                String fieldName = cls.getSimpleName() + "." + field.getName();
                int canId = field.getInt(null);
                CanId.Type deviceType = annotation.value();
                CanIdAndType canIdAndType = new CanIdAndType(deviceType, canId);
                canIdToFields.computeIfAbsent(canIdAndType, _canIdAndType -> new HashSet<>())
                        .add(fieldName);
            }
        }
        // Check subclasses recursively.
        for (Class<?> subClass : cls.getClasses()) {
            findCanIds(canIdToFields, subClass);
        }
    }
}
