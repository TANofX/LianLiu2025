package frc.robot;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.util.*;

public class ConstantsTest {

    /**
     * Test that no two CAN ID fields have the same CAN ID.
     */
    @Test
    public void testDuplicateCanIds() throws Exception {
        Map<Integer,Set<String>> canIdToFields = new HashMap<>();

        // find CAN IDs
        findCanIds(canIdToFields, Constants.class);

        // Verify that there are no duplicates
        List<String> errors = new ArrayList<>();
        canIdToFields.forEach((canId,fieldNames) -> {
            if (fieldNames.size() > 1) {
                String message = String.format("%s fields have CAN ID %s: %s", fieldNames.size(), canId, fieldNames);
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
    private void findCanIds(Map<Integer,Set<String>> canIdToFields, Class<?> cls)
        throws IllegalAccessException {
        // Find CAN ID fields
        for (Field field : cls.getDeclaredFields()) {
            Annotation annotation = field.getAnnotation(Constants.CanId.class);
            if (annotation != null) {
                String fieldName = cls.getSimpleName() + "." + field.getName();
                int canId = field.getInt(null);
                canIdToFields.computeIfAbsent(canId, _canId -> new HashSet<>())
                        .add(fieldName);
            }
        }
        // Check subclasses recursively.
        for (Class<?> subClass : cls.getClasses()) {
            findCanIds(canIdToFields, subClass);
        }
    }
}
