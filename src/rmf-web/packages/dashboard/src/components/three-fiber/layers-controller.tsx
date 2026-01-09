import { ChangeEvent } from 'react';
import { Level } from 'api-client';
import { AppEvents } from '../app-events';
import React from 'react';
import {
  Box,
  Checkbox,
  FormControl,
  FormControlLabel,
  FormGroup,
  IconButton,
  MenuItem,
  TextField,
  Paper,
  Typography,
  Divider,
  Collapse,
  styled,
} from '@mui/material';
import LayersIcon from '@mui/icons-material/Layers';
import ZoomInIcon from '@mui/icons-material/ZoomIn';
import ZoomOutIcon from '@mui/icons-material/ZoomOut';

interface LayersControllerProps {
  disabledLayers: Record<string, boolean>;
  onChange: (event: ChangeEvent<HTMLInputElement>, value: string) => void;
  levels: Level[];
  currentLevel: Level;
  handleZoomIn: () => void;
  handleZoomOut: () => void;
}

const ScrollableLayerList = styled(Box)({
  padding: '12px',
  maxHeight: '400px',
  overflowY: 'auto',
  '&::-webkit-scrollbar': {
    width: '6px',
  },
  '&::-webkit-scrollbar-track': {
    background: 'rgba(0, 0, 0, 0.1)',
    borderRadius: '3px',
  },
  '&::-webkit-scrollbar-thumb': {
    background: 'rgba(59, 130, 246, 0.5)',
    borderRadius: '3px',
    '&:hover': {
      background: 'rgba(59, 130, 246, 0.7)',
    },
  },
});

export const LayersController = ({
  disabledLayers,
  onChange,
  levels,
  currentLevel,
  handleZoomIn,
  handleZoomOut,
}: LayersControllerProps) => {
  const [isLayersOpen, setIsLayersOpen] = React.useState(false);

  return (
    <Box
      component="div"
      sx={{
        position: 'absolute',
        top: '70px',
        left: '12px',
        width: 'auto',
        height: 'auto',
        zIndex: 1000,
        display: 'flex',
        flexDirection: 'column',
        gap: 1,
      }}
    >
      {/* Level Selector */}
      <Paper
        elevation={3}
        sx={{
          background: 'rgba(255, 255, 255, 0.15)',
          backdropFilter: 'blur(15px)',
          border: '1px solid rgba(255, 255, 255, 0.2)',
          borderRadius: '12px',
          padding: '8px',
          transition: 'all 0.3s ease',
          '&:hover': {
            background: 'rgba(255, 255, 255, 0.2)',
            transform: 'translateY(-2px)',
            boxShadow: '0 10px 25px rgba(0, 0, 0, 0.2)',
          },
        }}
      >
        <FormControl fullWidth>
          <TextField
            select
            id="level-select"
            label="Level"
            variant="outlined"
            value={currentLevel.name}
            size="small"
            sx={{
              width: '120px',
              '& .MuiOutlinedInput-root': {
                borderRadius: '8px',
                backgroundColor: 'rgba(255, 255, 255, 0.1)',
              },
              '& .MuiInputLabel-root': {
                fontWeight: 600,
              },
            }}
            onChange={(e: ChangeEvent<HTMLInputElement>) => onChange(e, e.target.value as string)}
          >
            {levels.map((level, i) => (
              <MenuItem key={i} value={level.name}>
                {level.name}
              </MenuItem>
            ))}
          </TextField>
        </FormControl>
      </Paper>

      {/* Zoom Controls */}
      <Paper
        elevation={3}
        sx={{
          background: 'rgba(255, 255, 255, 0.15)',
          backdropFilter: 'blur(15px)',
          border: '1px solid rgba(255, 255, 255, 0.2)',
          borderRadius: '12px',
          padding: '4px',
          display: 'flex',
          flexDirection: 'column',
          gap: 0.5,
        }}
      >
        <IconButton
          size="small"
          onClick={handleZoomIn}
          data-testid="zoom-in"
          sx={{
            transition: 'all 0.2s ease',
            '&:hover': {
              backgroundColor: 'rgba(59, 130, 246, 0.2)',
              transform: 'scale(1.1)',
            },
          }}
        >
          <ZoomInIcon fontSize="medium" />
        </IconButton>
        <Divider sx={{ borderColor: 'rgba(255, 255, 255, 0.2)' }} />
        <IconButton
          size="small"
          onClick={handleZoomOut}
          data-testid="zoom-out"
          sx={{
            transition: 'all 0.2s ease',
            '&:hover': {
              backgroundColor: 'rgba(59, 130, 246, 0.2)',
              transform: 'scale(1.1)',
            },
          }}
        >
          <ZoomOutIcon fontSize="medium" />
        </IconButton>
      </Paper>

      {/* Layers Toggle */}
      <Paper
        elevation={3}
        sx={{
          background: 'rgba(255, 255, 255, 0.15)',
          backdropFilter: 'blur(15px)',
          border: '1px solid rgba(255, 255, 255, 0.2)',
          borderRadius: '12px',
          overflow: 'hidden',
          transition: 'all 0.3s ease',
        }}
      >
        <IconButton
          size="small"
          data-testid="layers"
          onClick={() => setIsLayersOpen(!isLayersOpen)}
          sx={{
            width: '100%',
            borderRadius: '12px',
            padding: '8px',
            transition: 'all 0.2s ease',
            backgroundColor: isLayersOpen ? 'rgba(59, 130, 246, 0.2)' : 'transparent',
            '&:hover': {
              backgroundColor: 'rgba(59, 130, 246, 0.3)',
            },
          }}
        >
          <LayersIcon fontSize="medium" />
        </IconButton>

        <Collapse in={isLayersOpen}>
          <ScrollableLayerList>
            <Typography
              variant="caption"
              sx={{
                fontWeight: 700,
                color: 'rgba(0, 0, 0, 0.7)',
                marginBottom: 1,
                display: 'block',
                textTransform: 'uppercase',
                letterSpacing: '0.5px',
              }}
            >
              Map Layers
            </Typography>
            <FormGroup>
              {Object.keys(disabledLayers).map((layerName) => (
                <FormControlLabel
                  key={layerName}
                  control={
                    <Checkbox
                      size="small"
                      checked={!disabledLayers[layerName]}
                      onChange={() => {
                        const updatedLayers = { ...disabledLayers };
                        updatedLayers[layerName] = !updatedLayers[layerName];
                        AppEvents.disabledLayers.next(updatedLayers);
                      }}
                      sx={{
                        color: 'rgba(59, 130, 246, 0.7)',
                        '&.Mui-checked': {
                          color: 'rgba(59, 130, 246, 1)',
                        },
                        transition: 'all 0.2s ease',
                      }}
                    />
                  }
                  label={
                    <Typography
                      variant="body2"
                      sx={{
                        fontSize: '0.875rem',
                        fontWeight: 500,
                        color: !disabledLayers[layerName]
                          ? 'rgba(0, 0, 0, 0.87)'
                          : 'rgba(0, 0, 0, 0.5)',
                      }}
                    >
                      {layerName}
                    </Typography>
                  }
                  sx={{
                    marginBottom: 0.5,
                    padding: '4px 8px',
                    borderRadius: '6px',
                    transition: 'all 0.2s ease',
                    '&:hover': {
                      backgroundColor: 'rgba(59, 130, 246, 0.1)',
                    },
                  }}
                />
              ))}
            </FormGroup>
          </ScrollableLayerList>
        </Collapse>
      </Paper>
    </Box>
  );
};
