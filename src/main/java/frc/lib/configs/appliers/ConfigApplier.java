package frc.lib.configs.appliers;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

/** Applies configs */
public class ConfigApplier {
  
  /**
   * Attempts to apply a config. Returns true if successful.
   *
   * @param applier a function that attempts to apply a config. Returns the result of the
   *     application.
   * @param isSuccess a function that returns true if the result of an application is a success.
   * @param retries the number of unsuccessful attempts before failing.
   * @return true if successful.
   */
  protected static <Result> boolean attempt(
      Supplier<Result> applier, Function<Result, Boolean> isSuccess, int retries) {
    
    for (int i = 0; i < retries; i++) {
      Result result = applier.get();

      if (isSuccess.apply(result)) {
        return true;
      }
    }

    return false;
  }

  /**
   * Attempts to apply a Phoenix 6 config. Returns true if successful.
   *
   * @param applier a function that attempts to apply a config. Returns the result of the
   *     application.
   * @return true if successful.
   */
  protected static boolean attempt(Supplier<StatusCode> applier) {
    return attempt(() -> applier.get(), StatusCode::isOK, 10);
  }
}
