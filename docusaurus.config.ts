import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Mike Likes Robots',
  tagline: 'A place for tutorials and sharing information so everyone can build better robots!',
  favicon: 'favicon.ico',

  // Set the production url of your site here
  url: 'https://mikelikesrobots.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'mikelikesrobots', // Usually your GitHub org/user name.
  projectName: 'mikelikesrobots.github.io', // Usually your repo name.
  deploymentBranch: 'docusaurus',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts'
        },
        blog: {
          showReadingTime: true
        },
        theme: {
          customCss: './src/css/custom.css',
        },
        gtag: {
          trackingID: "G-7X3VQWMTBV"
        }
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Mike Likes Robots',
      logo: {
        alt: 'Mike Likes Robots',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'gettingStartedSidebar',
          position: 'left',
          label: 'Getting Started',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://community.aws/@mikelikesrobots',
          label: 'AWS Community',
          position: 'right',
        },
        {
          href: 'https://www.youtube.com/channel/UCAdis20vFM97syzwG1YlacQ',
          label: 'YouTube',
          position: 'right',
        },
        {
          href: 'https://www.linkedin.com/in/michael-hart-a7614262/',
          label: 'LinkedIn',
          position: 'right',
        },
        {
          href: 'https://github.com/mikelikesrobots',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Getting Started Guide',
              to: '/docs/getting-started/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              href: 'https://community.aws/@mikelikesrobots',
              label: 'AWS Community',
            },
            {
              href: 'https://www.youtube.com/channel/UCAdis20vFM97syzwG1YlacQ',
              label: 'YouTube',
            },
            {
              href: 'https://www.linkedin.com/in/michael-hart-a7614262/',
              label: 'LinkedIn',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'Flaticons - Freepik',
              to: 'https://www.flaticon.com/free-icons/start-button',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/facebook/docusaurus',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Mike Likes Robots. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
