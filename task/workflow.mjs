export function workflowFromPlan(plan) {
  const stages = Array.from(plan?.stages ?? []);
  const workflow = [];
  if (stages.some((stage) => stage.kind === 'open_door')) workflow.push('open');
  if (stages.some((stage) => stage.kind === 'grip_pick')) workflow.push('pick');
  if (stages.some((stage) => stage.kind === 'carry_inside')) workflow.push('place');
  return workflow;
}

export function trackedEntitiesFromPlan(plan) {
  const entities = plan?.entities ?? {};
  return {
    bodyNames: Array.from(new Set([
      entities.objectBodyName,
      entities.containerBodyName,
      entities.doorBodyName,
    ].filter(Boolean))),
    jointNames: Array.from(new Set([
      entities.doorJointName,
    ].filter(Boolean))),
  };
}
