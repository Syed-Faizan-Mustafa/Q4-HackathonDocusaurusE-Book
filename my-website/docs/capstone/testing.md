---
sidebar_position: 3
title: "Testing"
description: Test your VLA pipeline
---

# Testing

Verify your complete VLA system.

## Test Commands

```bash
# Test voice pipeline
ros2 topic echo /voice/transcription

# Test LLM output
ros2 topic echo /llm/action_plan

# Test robot status
ros2 topic echo /execution/status
```

## Summary

**[Continue to Deployment →](./deployment)**
