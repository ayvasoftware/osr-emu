const path = require('path');
const TerserPlugin = require('terser-webpack-plugin');

const base = {
  performance: {
    hints: false,
  },
  optimization: {
    minimizer: [
      new TerserPlugin({
        extractComments: false,
        terserOptions: {
          format: {
            comments: false,
          },
        },
      }),
    ]
  }
};

module.exports = [{
  entry: './osr-emu.js',
  mode: 'production',
  experiments: {
    outputModule: true,
  },
  output: {
    filename: 'osr-emu.dist.module.min.js',
    path: path.resolve(__dirname, 'dist'),
    library: {
      type: 'module',
    },
  },
  ...base
}, {
  entry: './osr-emu.js',
  mode: 'production',
  output: {
    filename: 'osr-emu.dist.min.js',
    path: path.resolve(__dirname, 'dist'),
    library: {
      type: 'window',
    },
  },
  ...base
}];
