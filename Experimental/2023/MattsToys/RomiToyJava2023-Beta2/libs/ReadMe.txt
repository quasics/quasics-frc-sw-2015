Project-external libraries can go here, and also need to be added to the
build.gradle file, in the "dependencies" definition.

Example:
  dependencies {
    . . . .
    implementation files('libs/JavaUtilityLib-Beta2.jar')
    . . . .
  }