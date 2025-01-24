
class Person {
    String name;
    int age;

    Person(String name, int age) {
        this.name = name;
        this.age = age;
    }

    public String toString() {
        return "Person: " + name + ", Age: " + age;
    }

    // Other methods and data for "Person" stuff....
}

class CollegeStudent extends Person {
    String studentId;
    String major;

    CollegeStudent(String name, int age, String studentId, String major) {
        super(name, age);
        this.studentId = studentId;
        this.major = major;
    }

    public String toString() {
        return "CollegeStudent: " + name + ", Age: " + age + ", Student ID: " + studentId + ", Major: " + major;
    }

    // Other methods and data for "CollegeStudent" stuff....
}

class HighSchoolStudent extends Person {
    String studentId;
    String gradeLevel;

    HighSchoolStudent(String name, int age, String studentId, String gradeLevel) {
        super(name, age);
        this.studentId = studentId;
        this.gradeLevel = gradeLevel;
    }

    public String toString() {
        return "HighSchoolStudent: " + name + ", Age: " + age + ", Student ID: " + studentId + ", Grade Level: "
                + gradeLevel;
    }

    // Other methods and data for "HighSchoolStudent" stuff....
}

public class InheritanceDemo {
    public static void showPersonInfo(Person person) {
        System.out.println(person.toString());
    }

    public static void main(String[] args) {
        Person person = new Person("John Doe", 30);
        CollegeStudent collegeStudent = new CollegeStudent("Alice", 20, "CS123", "Computer Science");
        HighSchoolStudent highSchoolStudent = new HighSchoolStudent("Bob", 16, "HS456", "10th Grade");

        System.out.println(person);
        System.out.println(collegeStudent);
        System.out.println(highSchoolStudent);
    }
}
