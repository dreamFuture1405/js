const maximumJointError = (target, measured) => {
  if (!Array.isArray(target) || !Array.isArray(measured)
    || target.length === 0 || target.length !== measured.length) {
    throw new Error('Target and measured joint dimensions must match');
  }
  const differences = target.map((value, index) =>
    Math.abs(Number(value) - Number(measured[index])));
  if (!differences.every(Number.isFinite)) {
    throw new Error('Target and measured joints must be finite');
  }
  return Math.max(...differences);
};

export function evaluateJointTargetProgress({
  target,
  measured,
  bestError = Number.POSITIVE_INFINITY,
  noProgressCount = 0,
  elapsedMilliseconds = 0,
  targetToleranceRadians = 0.028,
  minimumProgressRadians = 0.0015,
  noProgressLimit = 12,
  maximumTargetMilliseconds = 1800,
}) {
  const errorRadians = maximumJointError(target, measured);
  const reached = errorRadians <= Math.max(0.001, Number(targetToleranceRadians));
  const improved = reached
    || !Number.isFinite(Number(bestError))
    || errorRadians <= Number(bestError) - Math.max(0, Number(minimumProgressRadians));
  const nextBestError = Math.min(Number(bestError), errorRadians);
  const nextNoProgressCount = improved
    ? 0
    : Math.max(0, Number(noProgressCount)) + 1;
  let stopReason = null;
  if (!reached && Number(elapsedMilliseconds) >= Number(maximumTargetMilliseconds)) {
    stopReason = 'TARGET_TIMEOUT';
  } else if (!reached && nextNoProgressCount >= Number(noProgressLimit)) {
    stopReason = 'TARGET_STALLED';
  }
  return {
    reached,
    improved,
    errorRadians,
    bestError: nextBestError,
    noProgressCount: nextNoProgressCount,
    stopReason,
  };
}

export { maximumJointError };
