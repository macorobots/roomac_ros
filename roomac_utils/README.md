# roomac_utils

Package containing common functionalities, that can be used in other places.

## action_procedure_executor

Provides `SimpleActionServer` implementation - `SimpleActionExecutor` that consists of list of multiple `ActionProcedureStep` that are run upon execution. Each step can be non blocking, providing feedback and a mean to cancel them, which is useful in can of longer steps (or a step that calls another action).
