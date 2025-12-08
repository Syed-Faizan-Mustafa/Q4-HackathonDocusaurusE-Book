import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Physical AI & Humanoid Robotics E-Book Sidebar Configuration
 * Manual configuration for controlled learning path progression
 */
const sidebars: SidebarsConfig = {
  learningSidebar: [
    'intro',
    'prerequisites',
    'learning-path',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      link: {
        type: 'doc',
        id: 'module-1-ros2/intro',
      },
      collapsed: false,
      items: [
        'module-1-ros2/concepts',
        'module-1-ros2/setup',
        'module-1-ros2/hello-ros2',
        'module-1-ros2/pubsub',
        'module-1-ros2/services',
        'module-1-ros2/actions',
        'module-1-ros2/urdf-basics',
        'module-1-ros2/troubleshooting',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twins (Gazebo)',
      link: {
        type: 'doc',
        id: 'module-2-gazebo/intro',
      },
      collapsed: true,
      items: [
        'module-2-gazebo/gazebo-basics',
        'module-2-gazebo/worlds',
        'module-2-gazebo/urdf-to-sim',
        'module-2-gazebo/physics-tuning',
        'module-2-gazebo/plugins',
        'module-2-gazebo/troubleshooting',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Isaac Sim & Navigation',
      link: {
        type: 'doc',
        id: 'module-3-isaac/intro',
      },
      collapsed: true,
      items: [
        'module-3-isaac/isaac-setup',
        'module-3-isaac/photorealistic',
        'module-3-isaac/isaac-ros',
        'module-3-isaac/vslam',
        'module-3-isaac/nav2',
        'module-3-isaac/troubleshooting',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA & AI Brain',
      link: {
        type: 'doc',
        id: 'module-4-vla/intro',
      },
      collapsed: true,
      items: [
        'module-4-vla/whisper',
        'module-4-vla/llm-planner',
        'module-4-vla/vla-pipeline',
        'module-4-vla/troubleshooting',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Hardware & Deployment',
      link: {
        type: 'doc',
        id: 'module-5-hardware/intro',
      },
      collapsed: true,
      items: [
        'module-5-hardware/workstation-specs',
        'module-5-hardware/jetson-deployment',
        'module-5-hardware/sensor-integration',
        'module-5-hardware/troubleshooting',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      link: {
        type: 'doc',
        id: 'capstone/overview',
      },
      collapsed: true,
      items: [
        'capstone/integration',
        'capstone/testing',
        'capstone/deployment',
        'capstone/demo-scripts',
      ],
    },
    {
      type: 'category',
      label: 'Appendix',
      collapsed: true,
      items: [
        'appendix/glossary',
        'appendix/ros2-cli-reference',
        'appendix/troubleshooting',
        'appendix/resources',
      ],
    },
  ],
};

export default sidebars;
