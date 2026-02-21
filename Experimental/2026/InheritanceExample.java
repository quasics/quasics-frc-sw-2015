// Define "Person" as a base class.
class Person {
  protected String firstName;
  protected String lastName;

  public Person(String first, String last) {
    firstName = first;
    lastName = last;
  }
  // Example: Person a = new Person("Sean", "McMahon");

  public String getName() {
    return firstName + " " + lastName;
  }
  // Example: a.getName() --> "Sean McMahon"
}

// Define "Faculty" as a derived class that is a "more specialized" version of
// "Person".
//
// This class is adding functionality ("getFamiliarName") that "Person" doesn't
// have (though it's *using* stuff from Person to do so), and overriding how
// some of the "Person" functionality works (by redefining "getName").
class Faculty extends Person {
  protected String title;

  public Faculty(String title, String first, String last) {
    super(first, last);
    this.title = title;
  }
  // Example: Faculty prof = new Faculty("Dr.", "Matthew", "Kohler");
  // Example: Person p = new Faculty("Dr.", "Vinnie", "Boombatz");

  public String getFamiliarName() {
    return firstName + " " + lastName;
  }

  public String getName() {
    return title + " " + super.getName();
  }
  // prof.getName() --> "Dr. Matthew Kohler"
  // p.getName() --> "Dr. Vinnie Boombatz"
}

// Ignore this class: it's just here to match the name of the file....
public class InheritanceExample {
  public static void main(String[] args) {
    Person a = new Person("Sean", "McMahon");
    Person p = new Faculty("Professor", "Samantha", "Stephens");
    Faculty f = new Faculty("Dr", "Matthew", "Kohler");

    System.out.println("Person a's preferred name is " + a.getName());
    System.out.println("Person (actually a Faculty) p's preferred name is " + p.getName());
    System.out.println("Faculty f's preferred name is " + f.getName());
  }
}
