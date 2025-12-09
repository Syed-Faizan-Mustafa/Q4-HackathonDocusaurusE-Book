import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// Physical AI & Humanoid Robotics E-Book Configuration
const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From ROS 2 Fundamentals to Vision-Language-Action Systems',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // GitHub Pages deployment
  url: 'https://faizanmustafa.github.io',
  baseUrl: '/e-book-hackathon-2025/',
  organizationName: 'faizanmustafa',
  projectName: 'e-book-hackathon-2025',
  trailingSlash: false,

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  markdown: {
    mermaid: true,
  },

  themes: ['@docusaurus/theme-mermaid'],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.ts'),
          editUrl:
            'https://github.com/faizanmustafa/e-book-hackathon-2025/tree/main/my-website/',
          showLastUpdateTime: false,
          showLastUpdateAuthor: false,
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/physical-ai-social-card.jpg',
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI E-Book',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'learningSidebar',
          position: 'left',
          label: 'Learn',
        },
        {to: '/docs/module-1-ros2/intro', label: 'ROS 2', position: 'left'},
        {to: '/docs/module-4-vla/intro', label: 'VLA', position: 'left'},
        {to: '/docs/capstone/overview', label: 'Capstone', position: 'left'},
        {
          href: 'https://github.com/faizanmustafa/e-book-hackathon-2025',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learning Modules',
          items: [
            {label: 'ROS 2 Fundamentals', to: '/docs/module-1-ros2/intro'},
            {label: 'Gazebo Simulation', to: '/docs/module-2-gazebo/intro'},
            {label: 'Isaac Sim', to: '/docs/module-3-isaac/intro'},
            {label: 'VLA & AI Brain', to: '/docs/module-4-vla/intro'},
            {label: 'Hardware', to: '/docs/module-5-hardware/intro'},
          ],
        },
        {
          title: 'Resources',
          items: [
            {label: 'Prerequisites', to: '/docs/prerequisites'},
            {label: 'Learning Path', to: '/docs/learning-path'},
            {label: 'Capstone Project', to: '/docs/capstone/overview'},
            {label: 'Glossary', to: '/docs/appendix/glossary'},
          ],
        },
        {
          title: 'Community',
          items: [
            {label: 'ROS Discourse', href: 'https://discourse.ros.org/'},
            {
              label: 'NVIDIA Isaac Forum',
              href:
                'https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/',
            },
            {
              label: 'GitHub Discussions',
              href:
                'https://github.com/faizanmustafa/e-book-hackathon-2025/discussions',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI E-Book. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['bash', 'python', 'yaml', 'markup', 'cpp', 'json'],
    },
    mermaid: {theme: {light: 'neutral', dark: 'dark'}},
    tableOfContents: {minHeadingLevel: 2, maxHeadingLevel: 4},
  } satisfies Preset.ThemeConfig,
};

export default config;
